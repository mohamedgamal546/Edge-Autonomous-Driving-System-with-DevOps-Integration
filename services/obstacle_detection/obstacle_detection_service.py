"""
Obstacle Detection Service
===========================
Dual-mode obstacle detection:
  1. Ultrasonic (HC-SR04) via Raspberry Pi GPIO  → short-range (< 50 cm) physical obstacles
  2. OpenCV HSV colour masking                   → medium-range visual obstacles (green objects)

Both sensors publish to the same MQTT topic so the control service
has a unified view of the road ahead.

MQTT Topics:
    Subscribes: cv/camera/frame
    Publishes:  system/obstacle     → obstacle detection result
                system/obstacle/health → service health
"""

import time
import base64
import logging
import json
import os
import threading
import numpy as np
import cv2
import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion
from prometheus_client import start_http_server, Counter, Gauge, Histogram

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [OBSTACLE] %(levelname)s - %(message)s"
)
logger = logging.getLogger("obstacle_detection")

# ── Prometheus Metrics ────────────────────────────────────────────────────────
OBSTACLES_DETECTED   = Counter("obstacle_detected_total",          "Total obstacle detection events")
ULTRASONIC_TRIGGERS  = Counter("obstacle_ultrasonic_triggers_total","Ultrasonic range alerts")
VISUAL_TRIGGERS      = Counter("obstacle_visual_triggers_total",    "Visual (camera) obstacle alerts")
DISTANCE_CM          = Gauge(  "obstacle_distance_cm",              "Latest ultrasonic distance (cm)")
OBSTACLE_PRESENT     = Gauge(  "obstacle_present",                  "1 if obstacle detected, 0 otherwise")
PROC_TIME            = Histogram("obstacle_processing_seconds",     "Frame processing latency")

# ── Config ────────────────────────────────────────────────────────────────────
MQTT_BROKER  = os.getenv("MQTT_BROKER",   "mosquitto")
MQTT_PORT    = int(os.getenv("MQTT_PORT", "1883"))
METRICS_PORT = int(os.getenv("METRICS_PORT", "8003"))

# Ultrasonic GPIO (BCM numbering)
TRIG_PIN     = int(os.getenv("ULTRASONIC_TRIG", "23"))
ECHO_PIN     = int(os.getenv("ULTRASONIC_ECHO", "24"))
DANGER_DIST  = float(os.getenv("DANGER_DISTANCE_CM", "30.0"))  # Stop threshold
WARN_DIST    = float(os.getenv("WARN_DISTANCE_CM",   "60.0"))  # Slow-down threshold

# Visual obstacle detection (green object HSV range)
GREEN_H_LOW  = int(os.getenv("GREEN_H_LOW",  "35"))
GREEN_H_HIGH = int(os.getenv("GREEN_H_HIGH", "85"))
GREEN_S_LOW  = int(os.getenv("GREEN_S_LOW",  "80"))
GREEN_S_HIGH = int(os.getenv("GREEN_S_HIGH", "255"))
GREEN_V_LOW  = int(os.getenv("GREEN_V_LOW",  "80"))
GREEN_V_HIGH = int(os.getenv("GREEN_V_HIGH", "255"))
MIN_AREA     = int(os.getenv("MIN_OBSTACLE_AREA", "800"))

# ROI for visual detection (centre lower half of frame)
ROI_TOP_PCT  = float(os.getenv("OBS_ROI_TOP",  "0.45"))
ROI_BOT_PCT  = float(os.getenv("OBS_ROI_BOT",  "0.85"))
ROI_LEFT_PCT = float(os.getenv("OBS_ROI_LEFT", "0.20"))
ROI_RIGHT_PCT= float(os.getenv("OBS_ROI_RIGHT","0.80"))

TOPIC_FRAME   = "cv/camera/frame"
TOPIC_OBSTACLE= "system/obstacle"
TOPIC_HEALTH  = "system/obstacle/health"


class UltrasonicSensor:
    """
    HC-SR04 driver for Raspberry Pi.
    Mounted under the camera for detecting close physical obstacles.
    Falls back to simulation mode (random distances) when GPIO is unavailable.
    """

    SPEED_OF_SOUND = 34300  # cm/s at 20°C

    def __init__(self, trig: int, echo: int):
        self.trig = trig
        self.echo = echo
        self._gpio = None
        self._sim  = False

        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(trig, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(echo, GPIO.IN)
            self._gpio = GPIO
            logger.info(f"Ultrasonic sensor ready → TRIG={trig}, ECHO={echo}")
        except (ImportError, RuntimeError):
            self._sim = True
            logger.warning("RPi.GPIO unavailable — ultrasonic in simulation mode")

    def measure_cm(self) -> float:
        """Return distance in centimetres (single measurement)."""
        if self._sim:
            # Return a safe simulated distance for dev/testing
            return 200.0

        GPIO = self._gpio
        # Trigger pulse
        GPIO.output(self.trig, GPIO.HIGH)
        time.sleep(0.00001)   # 10 µs pulse
        GPIO.output(self.trig, GPIO.LOW)

        timeout = time.time() + 0.04   # 40 ms max wait

        # Wait for echo HIGH
        t_start = time.time()
        while GPIO.input(self.echo) == 0:
            t_start = time.time()
            if t_start > timeout:
                return -1.0   # Timeout — no echo received

        # Wait for echo LOW
        t_end = time.time()
        while GPIO.input(self.echo) == 1:
            t_end = time.time()
            if t_end > timeout:
                return -1.0

        elapsed  = t_end - t_start
        distance = (elapsed * self.SPEED_OF_SOUND) / 2
        return round(distance, 2)

    def measure_median(self, samples: int = 3) -> float:
        """Take multiple readings and return the median (reduces noise)."""
        readings = []
        for _ in range(samples):
            d = self.measure_cm()
            if d > 0:
                readings.append(d)
            time.sleep(0.01)
        return float(np.median(readings)) if readings else -1.0

    def cleanup(self):
        if self._gpio:
            self._gpio.cleanup([self.trig, self.echo])


class VisualObstacleDetector:
    """
    Detects green obstacles using HSV colour masking within a road-level ROI.
    Works in tandem with the ultrasonic sensor for redundancy.
    """

    def detect(self, frame: np.ndarray) -> tuple[bool, float, np.ndarray]:
        """
        Returns (obstacle_found, max_contour_area, annotated_frame).
        """
        h, w = frame.shape[:2]
        y1 = int(h * ROI_TOP_PCT)
        y2 = int(h * ROI_BOT_PCT)
        x1 = int(w * ROI_LEFT_PCT)
        x2 = int(w * ROI_RIGHT_PCT)

        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            return False, 0.0, frame

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower = np.array([GREEN_H_LOW,  GREEN_S_LOW,  GREEN_V_LOW])
        upper = np.array([GREEN_H_HIGH, GREEN_S_HIGH, GREEN_V_HIGH])
        mask  = cv2.inRange(hsv, lower, upper)

        # Morphological clean-up to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        annotated  = frame.copy()
        detected   = False
        max_area   = 0.0

        # Draw ROI box
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (200, 200, 0), 1)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > MIN_AREA:
                detected  = True
                max_area  = max(max_area, area)
                # Offset contour coordinates back to full frame
                cnt_shifted = cnt + np.array([x1, y1])
                cv2.drawContours(annotated, [cnt_shifted], -1, (0, 255, 0), 2)
                rx, ry, rw, rh = cv2.boundingRect(cnt)
                cv2.rectangle(annotated,
                              (x1 + rx, y1 + ry),
                              (x1 + rx + rw, y1 + ry + rh),
                              (0, 255, 0), 2)
                cv2.putText(annotated, f"OBSTACLE  area={int(area)}",
                            (x1 + rx, y1 + ry - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        return detected, max_area, annotated


class ObstacleDetectionService:
    def __init__(self):
        self.ultrasonic = UltrasonicSensor(TRIG_PIN, ECHO_PIN)
        self.visual     = VisualObstacleDetector()
        self._start_time = time.time()
        self._latest_distance = 999.0
        self._ultrasonic_thread = None

        self.client = mqtt.Client(CallbackAPIVersion.VERSION1, client_id="obstacle_detection_service", clean_session=True)
        self.client.will_set(TOPIC_HEALTH,
                             json.dumps({"status": "offline", "service": "obstacle"}),
                             retain=True)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_frame

    # ── MQTT ──────────────────────────────────────────────────────────────────
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            logger.info(f"Connected to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
            client.subscribe(TOPIC_FRAME, qos=0)
        else:
            logger.error(f"MQTT connect failed (rc={rc})")

    def _on_frame(self, client, userdata, msg):
        t0 = time.time()
        try:
            data  = json.loads(msg.payload)
            jpg   = base64.b64decode(data["frame"])
            arr   = np.frombuffer(jpg, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                return

            # Visual detection
            vis_detected, vis_area, annotated = self.visual.detect(frame)

            # Ultrasonic reading (already running in background thread)
            distance   = self._latest_distance
            us_danger  = 0 < distance <= DANGER_DIST
            us_warning = DANGER_DIST < distance <= WARN_DIST

            # Combined status
            obstacle_detected = vis_detected or us_danger
            severity = "NONE"
            if us_danger or vis_detected:
                severity = "STOP"
                OBSTACLES_DETECTED.inc()
                if us_danger:
                    ULTRASONIC_TRIGGERS.inc()
                if vis_detected:
                    VISUAL_TRIGGERS.inc()
            elif us_warning:
                severity = "SLOW"

            OBSTACLE_PRESENT.set(1 if obstacle_detected else 0)
            DISTANCE_CM.set(distance if distance > 0 else 0)

            # HUD annotation
            dist_color = (0, 0, 255) if us_danger else (0, 165, 255) if us_warning else (0, 255, 0)
            cv2.putText(annotated, f"Ultrasonic: {distance:.1f} cm",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, dist_color, 2)
            cv2.putText(annotated, f"Status: {severity}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, dist_color, 2)

            _, jpeg  = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 70])
            debug_b64 = base64.b64encode(jpeg).decode("utf-8")

            payload = {
                "obstacle_detected":  obstacle_detected,
                "severity":           severity,          # "NONE" | "SLOW" | "STOP"
                "visual_detected":    vis_detected,
                "visual_area":        round(vis_area, 1),
                "ultrasonic_cm":      distance,
                "us_danger":          us_danger,
                "us_warning":         us_warning,
                "debug_frame":        debug_b64,
                "timestamp":          data.get("timestamp", t0),
                "proc_ms":            round((time.time() - t0) * 1000, 2),
            }
            client.publish(TOPIC_OBSTACLE, json.dumps(payload), qos=0)
            PROC_TIME.observe(time.time() - t0)

        except Exception as exc:
            logger.error(f"Frame processing error: {exc}", exc_info=True)

    # ── Ultrasonic background loop ─────────────────────────────────────────────
    def _ultrasonic_loop(self):
        """Continuously poll the ultrasonic sensor in a background thread."""
        logger.info("Ultrasonic polling thread started")
        while self._running:
            dist = self.ultrasonic.measure_median(samples=3)
            if dist > 0:
                self._latest_distance = dist
            time.sleep(0.05)   # ~20 Hz polling

    # ── Service lifecycle ─────────────────────────────────────────────────────
    def run(self):
        logger.info("═══ Obstacle Detection Service starting ═══")
        start_http_server(METRICS_PORT)
        logger.info(f"Prometheus metrics → http://0.0.0.0:{METRICS_PORT}/metrics")

        for attempt in range(1, 11):
            try:
                self.client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
                break
            except Exception as exc:
                logger.error(f"MQTT connect attempt {attempt}/10: {exc}")
                time.sleep(3)

        self._running = True
        self._ultrasonic_thread = threading.Thread(
            target=self._ultrasonic_loop, daemon=True
        )
        self._ultrasonic_thread.start()

        self.client.loop_start()

        try:
            while True:
                self.client.publish(TOPIC_HEALTH, json.dumps({
                    "status":      "online",
                    "service":     "obstacle",
                    "distance_cm": self._latest_distance,
                    "uptime_s":    round(time.time() - self._start_time),
                }), retain=True)
                time.sleep(5)
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        logger.info("Stopping Obstacle Detection Service…")
        self._running = False
        self.ultrasonic.cleanup()
        self.client.publish(TOPIC_HEALTH,
                            json.dumps({"status": "offline", "service": "obstacle"}),
                            retain=True)
        self.client.loop_stop()
        self.client.disconnect()
        logger.info("Obstacle Detection Service stopped ✓")


if __name__ == "__main__":
    svc = ObstacleDetectionService()
    try:
        svc.run()
    except KeyboardInterrupt:
        svc.stop()

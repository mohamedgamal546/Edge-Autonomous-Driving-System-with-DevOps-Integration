"""
Traffic Light Detection Service
================================
Detects red and green traffic lights using HSV colour segmentation
within a configurable ROI at the top of the frame.

Decision logic:
    RED   → publish STOP command
    GREEN → publish GO command
    NONE  → publish UNKNOWN (control service maintains last state)

MQTT Topics:
    Subscribes: cv/camera/frame
    Publishes:  cv/traffic            → detection result + colour
                system/traffic/health → service health
"""

import cv2
import time
import base64
import logging
import json
import os
import numpy as np
import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion
from prometheus_client import start_http_server, Counter, Gauge, Histogram

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [TRAFFIC] %(levelname)s - %(message)s"
)
logger = logging.getLogger("traffic_light")

# ── Prometheus Metrics ────────────────────────────────────────────────────────
FRAMES_PROCESSED = Counter("traffic_frames_processed_total", "Total frames analysed")
RED_DETECTIONS   = Counter("traffic_red_detected_total",     "Red light detections")
GREEN_DETECTIONS = Counter("traffic_green_detected_total",   "Green light detections")
PROC_TIME        = Histogram("traffic_processing_seconds",   "Per-frame processing latency")
LIGHT_STATE      = Gauge("traffic_light_state",              "0=none, 1=red, 2=green")

# ── Config ────────────────────────────────────────────────────────────────────
MQTT_BROKER  = os.getenv("MQTT_BROKER",   "mosquitto")
MQTT_PORT    = int(os.getenv("MQTT_PORT", "1883"))
METRICS_PORT = int(os.getenv("METRICS_PORT", "8004"))

# ROI — upper-centre portion of the frame (where traffic lights appear)
ROI_TOP_PCT   = float(os.getenv("TL_ROI_TOP",   "0.05"))
ROI_BOT_PCT   = float(os.getenv("TL_ROI_BOT",   "0.40"))
ROI_LEFT_PCT  = float(os.getenv("TL_ROI_LEFT",  "0.30"))
ROI_RIGHT_PCT = float(os.getenv("TL_ROI_RIGHT", "0.70"))

MIN_BLOB_AREA = int(os.getenv("TL_MIN_AREA", "120"))
CONF_FRAMES   = int(os.getenv("TL_CONFIRM_FRAMES", "3"))   # Require N consecutive frames to confirm

TOPIC_FRAME   = "cv/camera/frame"
TOPIC_TRAFFIC = "cv/traffic"
TOPIC_HEALTH  = "system/traffic/health"


class TrafficLightDetector:
    """
    HSV-based colour detection for circular traffic light blobs.
    Requires N consecutive matching frames before confirming a state
    (reduces false positives from reflections / coloured objects).
    """

    # HSV ranges — tuned for typical LED traffic lights
    RED_RANGES = [
        (np.array([0,   120, 100]), np.array([10,  255, 255])),   # lower red
        (np.array([160, 120, 100]), np.array([180, 255, 255])),   # upper red (wrap-around)
    ]
    GREEN_RANGE = (
        np.array([38, 100, 80]),
        np.array([82, 255, 255])
    )

    def __init__(self):
        self._state_history: list[str] = []    # Rolling buffer of recent states

    def _mask_colour(self, hsv: np.ndarray, colour: str) -> np.ndarray:
        if colour == "red":
            m1 = cv2.inRange(hsv, *self.RED_RANGES[0])
            m2 = cv2.inRange(hsv, *self.RED_RANGES[1])
            return cv2.bitwise_or(m1, m2)
        elif colour == "green":
            return cv2.inRange(hsv, *self.GREEN_RANGE)
        return np.zeros(hsv.shape[:2], dtype=np.uint8)

    def _largest_blob_area(self, mask: np.ndarray) -> float:
        kernel   = np.ones((3, 3), np.uint8)
        clean    = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0.0
        return max(cv2.contourArea(c) for c in contours)

    def _confirm_state(self, raw_state: str) -> str:
        """Sliding window majority vote over last CONF_FRAMES readings."""
        self._state_history.append(raw_state)
        if len(self._state_history) > CONF_FRAMES * 2:
            self._state_history = self._state_history[-CONF_FRAMES * 2:]

        recent = self._state_history[-CONF_FRAMES:]
        if recent.count("RED")   == CONF_FRAMES:
            return "RED"
        if recent.count("GREEN") == CONF_FRAMES:
            return "GREEN"
        return "UNKNOWN"

    def detect(self, frame: np.ndarray) -> dict:
        h, w = frame.shape[:2]
        y1 = int(h * ROI_TOP_PCT);  y2 = int(h * ROI_BOT_PCT)
        x1 = int(w * ROI_LEFT_PCT); x2 = int(w * ROI_RIGHT_PCT)

        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            return self._empty_result(frame, x1, y1, x2, y2)

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        red_area   = self._largest_blob_area(self._mask_colour(hsv, "red"))
        green_area = self._largest_blob_area(self._mask_colour(hsv, "green"))

        raw_state = "UNKNOWN"
        if red_area > MIN_BLOB_AREA or green_area > MIN_BLOB_AREA:
            raw_state = "RED" if red_area >= green_area else "GREEN"

        confirmed = self._confirm_state(raw_state)

        # ── Annotate ───────────────────────────────────────────────────────
        annotated = frame.copy()
        box_color = {
            "RED":     (0, 0, 255),
            "GREEN":   (0, 255, 0),
            "UNKNOWN": (128, 128, 128),
        }[confirmed]

        cv2.rectangle(annotated, (x1, y1), (x2, y2), box_color, 2)
        cv2.putText(annotated,
                    f"Traffic: {confirmed}  R={int(red_area)} G={int(green_area)}",
                    (x1, max(y1 - 8, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, box_color, 2)

        if confirmed == "RED":
            cv2.putText(annotated, "STOP", (w // 2 - 40, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 255), 4)
        elif confirmed == "GREEN":
            cv2.putText(annotated, "GO",   (w // 2 - 25, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 255, 0), 4)

        _, jpeg   = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 70])
        debug_b64 = base64.b64encode(jpeg).decode("utf-8")

        return {
            "state":       confirmed,          # "RED" | "GREEN" | "UNKNOWN"
            "red_area":    round(red_area,   1),
            "green_area":  round(green_area, 1),
            "command":     "STOP" if confirmed == "RED" else
                           "GO"   if confirmed == "GREEN" else "HOLD",
            "debug_frame": debug_b64,
        }

    def _empty_result(self, frame, x1, y1, x2, y2) -> dict:
        annotated = frame.copy()
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (128, 128, 128), 1)
        _, jpeg = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 70])
        return {
            "state": "UNKNOWN", "red_area": 0, "green_area": 0,
            "command": "HOLD",
            "debug_frame": base64.b64encode(jpeg).decode("utf-8"),
        }


class TrafficLightService:
    def __init__(self):
        self.detector    = TrafficLightDetector()
        self._start_time = time.time()

        self.client = mqtt.Client(CallbackAPIVersion.VERSION1, client_id="traffic_light_service", clean_session=True)
        self.client.will_set(TOPIC_HEALTH,
                             json.dumps({"status": "offline", "service": "traffic"}),
                             retain=True)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_frame

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

            result = self.detector.detect(frame)
            FRAMES_PROCESSED.inc()

            if result["state"] == "RED":
                RED_DETECTIONS.inc()
                LIGHT_STATE.set(1)
            elif result["state"] == "GREEN":
                GREEN_DETECTIONS.inc()
                LIGHT_STATE.set(2)
            else:
                LIGHT_STATE.set(0)

            payload = {
                **result,
                "timestamp": data.get("timestamp", t0),
                "proc_ms":   round((time.time() - t0) * 1000, 2),
            }
            client.publish(TOPIC_TRAFFIC, json.dumps(payload), qos=0)
            PROC_TIME.observe(time.time() - t0)

        except Exception as exc:
            logger.error(f"Frame processing error: {exc}", exc_info=True)

    def run(self):
        logger.info("═══ Traffic Light Service starting ═══")
        start_http_server(METRICS_PORT)
        logger.info(f"Prometheus metrics → http://0.0.0.0:{METRICS_PORT}/metrics")

        for attempt in range(1, 11):
            try:
                self.client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
                break
            except Exception as exc:
                logger.error(f"MQTT connect attempt {attempt}/10: {exc}")
                time.sleep(3)

        self.client.loop_start()

        try:
            while True:
                self.client.publish(TOPIC_HEALTH, json.dumps({
                    "status":   "online",
                    "service":  "traffic",
                    "uptime_s": round(time.time() - self._start_time),
                }), retain=True)
                time.sleep(5)
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        logger.info("Stopping Traffic Light Service…")
        self.client.publish(TOPIC_HEALTH,
                            json.dumps({"status": "offline", "service": "traffic"}),
                            retain=True)
        self.client.loop_stop()
        self.client.disconnect()
        logger.info("Traffic Light Service stopped ✓")


if __name__ == "__main__":
    svc = TrafficLightService()
    try:
        svc.run()
    except KeyboardInterrupt:
        svc.stop()

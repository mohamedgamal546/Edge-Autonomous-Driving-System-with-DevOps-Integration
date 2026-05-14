"""
Camera Service
==============
Captures frames from Raspberry Pi Camera module or USB webcam.
Publishes encoded frames via MQTT for downstream processing services.
Handles LED auto-adjustment for stable low-light navigation.

MQTT Topics Published:
    cv/camera/frame   → JPEG-encoded frame (base64) + metadata
    system/camera/health → Service health status
"""

import cv2
import time
import base64
import logging
import json
import threading
import os
import numpy as np
import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion
from prometheus_client import start_http_server, Counter, Gauge, Histogram

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [CAMERA] %(levelname)s - %(message)s"
)
logger = logging.getLogger("camera_service")

# ── Prometheus Metrics ────────────────────────────────────────────────────────
FRAMES_PUBLISHED  = Counter("camera_frames_published_total",  "Total frames published to MQTT")
FRAME_ERRORS      = Counter("camera_frame_errors_total",       "Total frame capture failures")
FPS_GAUGE         = Gauge(  "camera_fps_current",              "Current capture FPS")
FRAME_SIZE_BYTES  = Histogram("camera_frame_size_bytes",       "Encoded frame size in bytes",
                              buckets=[5_000, 10_000, 20_000, 40_000, 80_000])
MQTT_PUBLISH_LAT  = Histogram("camera_mqtt_publish_seconds",   "MQTT publish latency (seconds)")
CAMERA_CONNECTED  = Gauge("camera_connected",                  "Camera connection status: 1=ok, 0=fail")
LED_STATE         = Gauge("camera_led_state",                  "LED on=1 / off=0")

# ── Environment Config ────────────────────────────────────────────────────────
MQTT_BROKER   = os.getenv("MQTT_BROKER",    "mosquitto")
MQTT_PORT     = int(os.getenv("MQTT_PORT",  "1883"))
CAMERA_INDEX  = int(os.getenv("CAMERA_INDEX", "0"))
TARGET_FPS    = int(os.getenv("TARGET_FPS",   "15"))
JPEG_QUALITY  = int(os.getenv("JPEG_QUALITY", "80"))
FRAME_WIDTH   = int(os.getenv("FRAME_WIDTH",  "640"))
FRAME_HEIGHT  = int(os.getenv("FRAME_HEIGHT", "480"))
LED_PIN       = int(os.getenv("LED_GPIO_PIN", "18"))
ENABLE_LED    = os.getenv("ENABLE_LED", "true").lower() == "true"
METRICS_PORT  = int(os.getenv("METRICS_PORT", "8001"))

TOPIC_FRAME   = "cv/camera/frame"
TOPIC_HEALTH  = "system/camera/health"


class LEDController:
    """
    Controls the white/IR LED mounted alongside the camera.
    Automatically activates in low-light conditions to stabilise
    lane detection and obstacle avoidance at night.
    """

    BRIGHTNESS_LOW  = 60    # Turn LED on below this average pixel value
    BRIGHTNESS_HIGH = 110   # Turn LED off above this value (hysteresis)

    def __init__(self, pin: int):
        self.pin     = pin
        self.enabled = False
        self._gpio   = None

        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
            self._gpio = GPIO
            logger.info(f"LED controller ready → GPIO {pin}")
        except (ImportError, RuntimeError):
            logger.warning("RPi.GPIO unavailable — LED control disabled (simulation mode)")

    def on(self):
        if self._gpio and not self.enabled:
            self._gpio.output(self.pin, self._gpio.HIGH)
            self.enabled = True
            LED_STATE.set(1)
            logger.debug("LED → ON")

    def off(self):
        if self._gpio and self.enabled:
            self._gpio.output(self.pin, self._gpio.LOW)
            self.enabled = False
            LED_STATE.set(0)
            logger.debug("LED → OFF")

    def auto_adjust(self, frame: np.ndarray):
        """Switch LED based on frame brightness (with hysteresis)."""
        brightness = float(np.mean(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)))
        if brightness < self.BRIGHTNESS_LOW:
            self.on()
        elif brightness > self.BRIGHTNESS_HIGH:
            self.off()

    def cleanup(self):
        self.off()
        if self._gpio:
            self._gpio.cleanup(self.pin)


class CameraService:
    """
    Main camera capture loop.
    Reads frames → applies LED correction → encodes → publishes to MQTT.
    """

    def __init__(self):
        self.cap     = None
        self.running = False
        self.led     = LEDController(LED_PIN) if ENABLE_LED else None
        self._fps_times: list[float] = []
        self._start_time = time.time()

        self.client = mqtt.Client(CallbackAPIVersion.VERSION1, client_id="camera_service", clean_session=True)
        self.client.will_set(TOPIC_HEALTH,
                             json.dumps({"status": "offline", "service": "camera"}),
                             retain=True)
        self.client.on_connect    = self._on_connect
        self.client.on_disconnect = self._on_disconnect

    # ── MQTT callbacks ────────────────────────────────────────────────────────
    def _on_connect(self, client, userdata, flags, rc):
        codes = {0: "OK", 1: "Bad protocol", 2: "Rejected client ID",
                 3: "Server unavailable", 4: "Bad credentials", 5: "Not authorised"}
        if rc == 0:
            logger.info(f"MQTT connected to {MQTT_BROKER}:{MQTT_PORT}")
        else:
            logger.error(f"MQTT connect failed: {codes.get(rc, rc)}")

    def _on_disconnect(self, client, userdata, rc):
        if rc != 0:
            logger.warning(f"Unexpected MQTT disconnect (rc={rc}) — will auto-reconnect")

    def _connect_mqtt(self):
        for attempt in range(1, 11):
            try:
                self.client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
                self.client.loop_start()
                return
            except Exception as exc:
                logger.error(f"MQTT connect attempt {attempt}/10 failed: {exc}")
                time.sleep(3)
        raise RuntimeError("Cannot reach MQTT broker after 10 attempts — aborting")

    # ── Camera helpers ────────────────────────────────────────────────────────
    def _open_camera(self) -> bool:
        if self.cap:
            self.cap.release()
        try:
            self.cap = cv2.VideoCapture(CAMERA_INDEX)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            ok, _ = self.cap.read()
            if ok:
                CAMERA_CONNECTED.set(1)
                logger.info(f"Camera opened (index={CAMERA_INDEX}, {FRAME_WIDTH}×{FRAME_HEIGHT})")
                return True
        except Exception as exc:
            logger.error(f"Camera open error: {exc}")
        CAMERA_CONNECTED.set(0)
        return False

    def _encode_frame(self, frame: np.ndarray) -> str:
        params = [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
        _, buf  = cv2.imencode(".jpg", frame, params)
        return base64.b64encode(buf).decode("utf-8")

    def _current_fps(self) -> float:
        now = time.time()
        self._fps_times = [t for t in self._fps_times if now - t < 2.0]
        self._fps_times.append(now)
        return len(self._fps_times) / 2.0

    # ── Main loop ─────────────────────────────────────────────────────────────
    def run(self):
        logger.info("═══ Camera Service starting ═══")
        start_http_server(METRICS_PORT)
        logger.info(f"Prometheus metrics → http://0.0.0.0:{METRICS_PORT}/metrics")

        self._connect_mqtt()

        if not self._open_camera():
            logger.warning("Initial camera open failed — will retry in main loop")

        self.running       = True
        frame_interval     = 1.0 / TARGET_FPS
        last_health_ts     = 0.0

        while self.running:
            t0 = time.time()

            # Camera reconnect guard
            if not (self.cap and self.cap.isOpened()):
                logger.warning("Camera lost — reconnecting…")
                CAMERA_CONNECTED.set(0)
                self._open_camera()
                time.sleep(1)
                continue

            ret, frame = self.cap.read()
            if not ret or frame is None:
                FRAME_ERRORS.inc()
                CAMERA_CONNECTED.set(0)
                time.sleep(0.05)
                continue

            CAMERA_CONNECTED.set(1)

            # Auto LED for low-light
            if self.led:
                self.led.auto_adjust(frame)

            # Encode and publish
            encoded  = self._encode_frame(frame)
            fps      = self._current_fps()
            FPS_GAUGE.set(fps)
            FRAME_SIZE_BYTES.observe(len(encoded))

            payload = json.dumps({
                "frame":     encoded,
                "timestamp": t0,
                "fps":       round(fps, 2),
                "led_on":    self.led.enabled if self.led else False,
            })

            t_pub = time.time()
            self.client.publish(TOPIC_FRAME, payload, qos=0)
            MQTT_PUBLISH_LAT.observe(time.time() - t_pub)
            FRAMES_PUBLISHED.inc()

            # Health heartbeat every 5 s
            if time.time() - last_health_ts > 5:
                self.client.publish(TOPIC_HEALTH, json.dumps({
                    "status":  "online",
                    "service": "camera",
                    "fps":     round(fps, 2),
                    "led_on":  self.led.enabled if self.led else False,
                    "uptime_s": round(time.time() - self._start_time),
                }), retain=True)
                last_health_ts = time.time()

            # FPS cap
            sleep_t = frame_interval - (time.time() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)

    def stop(self):
        logger.info("Stopping Camera Service…")
        self.running = False
        if self.led:
            self.led.cleanup()
        if self.cap:
            self.cap.release()
        self.client.publish(TOPIC_HEALTH,
                            json.dumps({"status": "offline", "service": "camera"}),
                            retain=True)
        self.client.loop_stop()
        self.client.disconnect()
        logger.info("Camera Service stopped ✓")


if __name__ == "__main__":
    svc = CameraService()
    try:
        svc.run()
    except KeyboardInterrupt:
        svc.stop()

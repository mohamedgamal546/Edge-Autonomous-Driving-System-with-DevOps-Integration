"""
Lane Detection Service
======================
Subscribes to camera frames, runs Canny edge detection + Hough Line Transform
to identify lane boundaries, computes lateral offset, and publishes steering
corrections for the control service.

Pipeline:
    Frame (BGR) → Grayscale → GaussianBlur → Canny → ROI Mask
    → HoughLinesP → Lane lines → Offset calculation → Publish

MQTT Topics:
    Subscribes: cv/camera/frame
    Publishes:  cv/lane            → lane detection result + offset
                system/lane/health → service health
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
    format="%(asctime)s [LANE] %(levelname)s - %(message)s"
)
logger = logging.getLogger("lane_detection")

# ── Prometheus Metrics ────────────────────────────────────────────────────────
FRAMES_PROCESSED = Counter("lane_frames_processed_total",  "Total frames processed")
LANE_DETECTED    = Counter("lane_detected_total",           "Frames where lane was detected")
LANE_LOST        = Counter("lane_lost_total",               "Frames where lane was not found")
OFFSET_GAUGE     = Gauge(  "lane_offset_normalized",        "Normalised lane offset (-1 to 1)")
PROC_TIME        = Histogram("lane_processing_seconds",     "Frame processing duration",
                             buckets=[0.01, 0.02, 0.04, 0.08, 0.16])

# ── Config ────────────────────────────────────────────────────────────────────
MQTT_BROKER  = os.getenv("MQTT_BROKER",   "mosquitto")
MQTT_PORT    = int(os.getenv("MQTT_PORT", "1883"))
METRICS_PORT = int(os.getenv("METRICS_PORT", "8002"))

TOPIC_FRAME  = "cv/camera/frame"
TOPIC_LANE   = "cv/lane"
TOPIC_HEALTH = "system/lane/health"

# ── Canny + Hough tuning (override via env for live calibration) ──────────────
CANNY_LOW    = int(os.getenv("CANNY_LOW",    "50"))
CANNY_HIGH   = int(os.getenv("CANNY_HIGH",   "150"))
HOUGH_THRESH = int(os.getenv("HOUGH_THRESH", "40"))
HOUGH_MIN_LEN= int(os.getenv("HOUGH_MIN_LEN","80"))
HOUGH_MAX_GAP= int(os.getenv("HOUGH_MAX_GAP","25"))
ROI_BOTTOM_PCT = float(os.getenv("ROI_BOTTOM_PCT", "0.95"))  # % from top where ROI starts
ROI_TOP_PCT    = float(os.getenv("ROI_TOP_PCT",    "0.55"))  # % from top where ROI ends


class LaneDetector:
    """
    Classical computer vision lane detection pipeline.
    Uses Canny edge detection + probabilistic Hough Line Transform.
    """

    def __init__(self, frame_w: int = 640, frame_h: int = 480):
        self.w = frame_w
        self.h = frame_h
        self.center_x   = frame_w // 2
        self._last_offset = 0.0     # Smoothed offset memory

    # ── ROI mask ──────────────────────────────────────────────────────────────
    def _build_roi_mask(self, edges: np.ndarray) -> np.ndarray:
        """
        Trapezoidal ROI focused on the road surface directly in front of the car.
        Filters out sky, horizon, and irrelevant background.
        """
        h, w = edges.shape
        top_y    = int(h * ROI_TOP_PCT)
        bottom_y = int(h * ROI_BOTTOM_PCT)
        top_w    = int(w * 0.45)   # Narrow top of trapezoid

        polygon = np.array([[
            (0,             bottom_y),
            (w,             bottom_y),
            (w // 2 + top_w, top_y),
            (w // 2 - top_w, top_y),
        ]], dtype=np.int32)

        mask = np.zeros_like(edges)
        cv2.fillPoly(mask, polygon, 255)
        return cv2.bitwise_and(edges, mask)

    # ── Line averaging ────────────────────────────────────────────────────────
    def _average_lines(self, lines) -> tuple[list, list]:
        """
        Separate detected Hough lines into left/right lane by slope,
        then fit a single averaged line for each side.
        """
        left_pts, right_pts = [], []

        if lines is None:
            return [], []

        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1:
                continue                        # skip vertical lines
            slope = (y2 - y1) / (x2 - x1)
            if abs(slope) < 0.3:
                continue                        # skip near-horizontal noise

            if slope < 0:                       # negative slope → left lane
                left_pts.extend([(x1, y1), (x2, y2)])
            else:                               # positive slope → right lane
                right_pts.extend([(x1, y1), (x2, y2)])

        def fit_line(pts: list) -> list:
            if len(pts) < 2:
                return []
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            poly = np.polyfit(ys, xs, 1)       # x = f(y) for vertical-ish lines
            y_bottom = int(self.h * ROI_BOTTOM_PCT)
            y_top    = int(self.h * ROI_TOP_PCT)
            x_bottom = int(np.polyval(poly, y_bottom))
            x_top    = int(np.polyval(poly, y_top))
            return [(x_bottom, y_bottom, x_top, y_top)]

        return fit_line(left_pts), fit_line(right_pts)

    # ── Offset calculation ────────────────────────────────────────────────────
    def _compute_offset(self, left_line: list, right_line: list) -> tuple[float, int]:
        """
        Compute normalized lateral offset from lane centre.
        Returns (normalized_offset ∈ [-1,1], raw_lane_center_x).
        Positive offset → car is right of centre (needs left correction).
        Negative offset → car is left of centre (needs right correction).
        """
        lane_center = self.center_x     # default: assume centered

        if left_line and right_line:
            lx = left_line[0][0]        # x at bottom of left line
            rx = right_line[0][0]       # x at bottom of right line
            lane_center = (lx + rx) // 2
        elif left_line:
            lane_center = left_line[0][0] + int(self.w * 0.25)
        elif right_line:
            lane_center = right_line[0][0] - int(self.w * 0.25)

        raw_offset  = lane_center - self.center_x
        norm_offset = raw_offset / (self.w / 2)   # normalise to [-1, 1]

        # Smooth with exponential moving average (α=0.3)
        alpha = 0.3
        smoothed = alpha * norm_offset + (1 - alpha) * self._last_offset
        self._last_offset = smoothed

        return round(smoothed, 4), lane_center

    # ── Main processing ───────────────────────────────────────────────────────
    def process(self, frame: np.ndarray) -> dict:
        """
        Full lane detection pipeline.
        Returns a result dict with offset, line coords, and an annotated frame.
        """
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur  = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, CANNY_LOW, CANNY_HIGH)
        roi   = self._build_roi_mask(edges)

        lines = cv2.HoughLinesP(
            roi,
            rho=1, theta=np.pi / 180,
            threshold=HOUGH_THRESH,
            minLineLength=HOUGH_MIN_LEN,
            maxLineGap=HOUGH_MAX_GAP,
        )

        left_line, right_line = self._average_lines(lines)
        lane_found = bool(left_line or right_line)

        offset, lane_center = self._compute_offset(left_line, right_line)

        # ── Annotate frame ────────────────────────────────────────────────────
        annotated = frame.copy()
        overlay   = frame.copy()

        # Draw filled lane polygon between the two lines
        if left_line and right_line:
            lx1, ly1, lx2, ly2 = left_line[0]
            rx1, ry1, rx2, ry2 = right_line[0]
            poly_pts = np.array([[lx1, ly1], [lx2, ly2],
                                  [rx2, ry2], [rx1, ry1]], dtype=np.int32)
            cv2.fillPoly(overlay, [poly_pts], (0, 255, 0))
            cv2.addWeighted(overlay, 0.3, annotated, 0.7, 0, annotated)

        # Draw individual lane lines
        for line in (left_line or []):
            x1, y1, x2, y2 = line
            cv2.line(annotated, (x1, y1), (x2, y2), (255, 100, 0), 4)
        for line in (right_line or []):
            x1, y1, x2, y2 = line
            cv2.line(annotated, (x1, y1), (x2, y2), (0, 100, 255), 4)

        # Lane centre marker
        y_marker = int(self.h * ROI_BOTTOM_PCT) - 10
        cv2.circle(annotated, (lane_center, y_marker), 8, (0, 255, 255), -1)
        cv2.line(annotated, (self.center_x, y_marker - 15),
                             (self.center_x, y_marker + 15), (255, 255, 255), 2)

        # HUD text
        status  = "LANE OK" if lane_found else "LANE LOST"
        color   = (0, 255, 0) if lane_found else (0, 0, 255)
        side    = "LEFT" if offset < -0.1 else ("RIGHT" if offset > 0.1 else "CENTER")
        cv2.putText(annotated, f"{status} | Offset: {offset:+.3f} ({side})",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        _, jpeg = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 70])
        debug_b64 = base64.b64encode(jpeg).decode("utf-8")

        return {
            "lane_detected": lane_found,
            "offset":        offset,
            "lane_center_x": lane_center,
            "left_line":     left_line,
            "right_line":    right_line,
            "debug_frame":   debug_b64,
        }


class LaneDetectionService:
    def __init__(self):
        self.detector    = None
        self._start_time = time.time()
        self.client = mqtt.Client(CallbackAPIVersion.VERSION1, client_id="lane_detection_service", clean_session=True)
        self.client.will_set(TOPIC_HEALTH,
                             json.dumps({"status": "offline", "service": "lane"}),
                             retain=True)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_frame

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            logger.info(f"Connected to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
            client.subscribe(TOPIC_FRAME, qos=0)
            logger.info(f"Subscribed to {TOPIC_FRAME}")
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

            # Lazy-init detector on first real frame
            if self.detector is None:
                h, w = frame.shape[:2]
                self.detector = LaneDetector(frame_w=w, frame_h=h)
                logger.info(f"LaneDetector initialised ({w}×{h})")

            result = self.detector.process(frame)

            FRAMES_PROCESSED.inc()
            if result["lane_detected"]:
                LANE_DETECTED.inc()
            else:
                LANE_LOST.inc()
            OFFSET_GAUGE.set(result["offset"])

            payload = {
                **result,
                "timestamp": data.get("timestamp", t0),
                "proc_ms":   round((time.time() - t0) * 1000, 2),
            }
            client.publish(TOPIC_LANE, json.dumps(payload), qos=0)
            PROC_TIME.observe(time.time() - t0)

        except Exception as exc:
            logger.error(f"Frame processing error: {exc}", exc_info=True)

    def run(self):
        logger.info("═══ Lane Detection Service starting ═══")
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

        # Periodic health publish
        try:
            while True:
                self.client.publish(TOPIC_HEALTH, json.dumps({
                    "status":   "online",
                    "service":  "lane",
                    "uptime_s": round(time.time() - self._start_time),
                }), retain=True)
                time.sleep(5)
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        logger.info("Stopping Lane Detection Service…")
        self.client.publish(TOPIC_HEALTH,
                            json.dumps({"status": "offline", "service": "lane"}),
                            retain=True)
        self.client.loop_stop()
        self.client.disconnect()
        logger.info("Lane Detection Service stopped ✓")


if __name__ == "__main__":
    svc = LaneDetectionService()
    try:
        svc.run()
    except KeyboardInterrupt:
        svc.stop()

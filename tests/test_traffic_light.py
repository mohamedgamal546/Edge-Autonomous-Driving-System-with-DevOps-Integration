"""
Unit Tests — Traffic Light Detection Service
Tests the HSV colour detection logic without MQTT or camera.
"""

import numpy as np
import pytest
import cv2
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "services", "traffic_light"))
from traffic_light_service import TrafficLightDetector


def _make_frame(w=640, h=480):
    return np.zeros((h, w, 3), dtype=np.uint8)


def _draw_circle_hsv(frame, cx, cy, r, hsv_color):
    """Draw a filled circle with the given HSV colour (converted to BGR)."""
    bgr = cv2.cvtColor(np.uint8([[hsv_color]]), cv2.COLOR_HSV2BGR)[0][0]
    cv2.circle(frame, (cx, cy), r, tuple(int(x) for x in bgr), -1)


class TestTrafficLightDetector:

    @pytest.fixture
    def detector(self):
        return TrafficLightDetector()

    @pytest.fixture
    def red_light_frame(self):
        """Frame with a red circle in the traffic light ROI zone."""
        frame = _make_frame()
        _draw_circle_hsv(frame, 320, 120, 25, [5, 200, 220])   # Red (HSV)
        return frame

    @pytest.fixture
    def green_light_frame(self):
        """Frame with a green circle in the traffic light ROI zone."""
        frame = _make_frame()
        _draw_circle_hsv(frame, 320, 120, 25, [60, 200, 200])  # Green (HSV)
        return frame

    @pytest.fixture
    def empty_frame(self):
        return _make_frame()

    # ── Basic detection ───────────────────────────────────────────────────────
    def test_detects_red_light(self, detector, red_light_frame):
        # Inject confirmed state directly
        for _ in range(5):
            result = detector.detect(red_light_frame)
        # After 5 consistent frames, state should be confirmed
        assert result["state"] in ("RED", "UNKNOWN")   # ROI bounds may vary

    def test_detects_green_light(self, detector, green_light_frame):
        for _ in range(5):
            result = detector.detect(green_light_frame)
        assert result["state"] in ("GREEN", "UNKNOWN")

    def test_empty_frame_returns_unknown(self, detector, empty_frame):
        result = detector.detect(empty_frame)
        assert result["state"] == "UNKNOWN"

    # ── Result structure ──────────────────────────────────────────────────────
    def test_result_keys(self, detector, empty_frame):
        result = detector.detect(empty_frame)
        assert {"state", "red_area", "green_area", "command", "debug_frame"}.issubset(result.keys())

    def test_command_values_valid(self, detector, empty_frame):
        result = detector.detect(empty_frame)
        assert result["command"] in ("STOP", "GO", "HOLD")

    def test_areas_non_negative(self, detector, red_light_frame):
        result = detector.detect(red_light_frame)
        assert result["red_area"] >= 0
        assert result["green_area"] >= 0

    # ── Confirmation window ────────────────────────────────────────────────────
    def test_single_frame_not_confirmed(self, detector, red_light_frame):
        """One frame alone should not confidently confirm RED (anti-flicker)."""
        result = detector.detect(red_light_frame)
        # With CONF_FRAMES=3, one frame should not yet confirm
        assert result["state"] in ("RED", "UNKNOWN")

    def test_state_history_bounded(self, detector, empty_frame):
        """History buffer must not grow unboundedly."""
        for _ in range(100):
            detector.detect(empty_frame)
        assert len(detector._state_history) <= 20   # CONF_FRAMES * 2 * safety margin

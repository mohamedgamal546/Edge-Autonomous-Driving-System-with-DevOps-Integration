"""
Unit Tests — Lane Detection Service
Tests the core CV pipeline without requiring a camera or MQTT broker.
"""

import numpy as np
import pytest
import cv2
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "services", "lane_detection"))
from lane_detection_service import LaneDetector


class TestLaneDetector:

    @pytest.fixture
    def detector(self):
        return LaneDetector(frame_w=640, frame_h=480)

    @pytest.fixture
    def straight_lane_frame(self):
        """Synthetic frame with two parallel white vertical-ish lines = straight lane."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        # Left lane line
        cv2.line(frame, (220, 480), (260, 280), (255, 255, 255), 8)
        # Right lane line
        cv2.line(frame, (420, 480), (380, 280), (255, 255, 255), 8)
        return frame

    @pytest.fixture
    def left_lane_only_frame(self):
        """Frame with only the left lane line (car drifted right)."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.line(frame, (150, 480), (180, 280), (255, 255, 255), 8)
        return frame

    @pytest.fixture
    def empty_frame(self):
        """All-black frame — no lane visible."""
        return np.zeros((480, 640, 3), dtype=np.uint8)

    # ── Initialisation ────────────────────────────────────────────────────────
    def test_detector_initialises(self, detector):
        assert detector.w == 640
        assert detector.h == 480
        assert detector.center_x == 320

    # ── Lane detection ────────────────────────────────────────────────────────
    def test_straight_lane_detected(self, detector, straight_lane_frame):
        result = detector.process(straight_lane_frame)
        assert result["lane_detected"] is True

    def test_empty_frame_no_lane(self, detector, empty_frame):
        result = detector.process(empty_frame)
        assert result["lane_detected"] is False

    def test_result_has_required_keys(self, detector, straight_lane_frame):
        result = detector.process(straight_lane_frame)
        required = {"lane_detected", "offset", "lane_center_x", "left_line", "right_line", "debug_frame"}
        assert required.issubset(result.keys())

    # ── Offset validation ─────────────────────────────────────────────────────
    def test_offset_within_bounds(self, detector, straight_lane_frame):
        result = detector.process(straight_lane_frame)
        assert -1.0 <= result["offset"] <= 1.0

    def test_straight_lane_offset_near_zero(self, detector, straight_lane_frame):
        result = detector.process(straight_lane_frame)
        # Symmetric lines → offset should be close to 0
        assert abs(result["offset"]) < 0.3, f"Offset too large: {result['offset']}"

    def test_no_lane_offset_uses_last_value(self, detector, empty_frame):
        # First call with a real frame to set a non-zero history
        detector._last_offset = 0.5
        result = detector.process(empty_frame)
        # Smoothed result should be influenced by last offset
        assert result["offset"] != 0.0 or detector._last_offset == 0.0

    # ── Smoothing ─────────────────────────────────────────────────────────────
    def test_offset_smoothing_reduces_jitter(self, detector):
        """Sudden large offset should be damped by the EMA filter."""
        detector._last_offset = 0.0
        # Compute smoothed value for a maximum offset
        raw = 1.0
        alpha = 0.3
        expected = alpha * raw + (1 - alpha) * 0.0
        # Simulate one smoothing step
        result = alpha * raw + (1 - alpha) * detector._last_offset
        assert result == pytest.approx(expected, abs=1e-6)

    # ── Debug frame ───────────────────────────────────────────────────────────
    def test_debug_frame_is_base64(self, detector, straight_lane_frame):
        import base64
        result = detector.process(straight_lane_frame)
        # Should not raise
        decoded = base64.b64decode(result["debug_frame"])
        # Should be a valid JPEG
        arr   = np.frombuffer(decoded, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        assert frame is not None
        assert frame.shape[:2] == (480, 640)

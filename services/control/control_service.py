"""
Control Service (Vehicle Brain)
================================
Subscribes to all perception topics and fuses their outputs into
a single prioritised motor command sent to the hardware:

    Priority (highest → lowest):
        1. Traffic light STOP / GO
        2. Obstacle STOP / SLOW
        3. Lane-following steering
        4. Path recovery (when lane is lost)

Hardware:
    • L298N motor driver → rear drive motors (speed via PWM)
    • Servo motor        → front steering (angle via PWM)

MQTT Topics Subscribed:
    cv/lane          ← lateral offset from lane centre
    cv/traffic       ← traffic light state (RED/GREEN/UNKNOWN)
    system/obstacle  ← obstacle presence + severity

MQTT Topics Published:
    control/steering  → current steering command (for logging/monitoring)
    system/control/health
"""

import time
import logging
import json
import os
import threading
import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion
from prometheus_client import start_http_server, Counter, Gauge, Histogram

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [CONTROL] %(levelname)s - %(message)s"
)
logger = logging.getLogger("control_service")

# ── Prometheus Metrics ────────────────────────────────────────────────────────
COMMANDS_SENT    = Counter("control_commands_sent_total",    "Total motor commands dispatched")
STOPS_TOTAL      = Counter("control_stops_total",            "Total stop events")
RECOVERY_EVENTS  = Counter("control_recovery_total",         "Path recovery activations")
SPEED_GAUGE      = Gauge(  "control_speed_pwm",              "Current drive speed (PWM 0-100)")
STEERING_GAUGE   = Gauge(  "control_steering_angle",         "Current steering angle (degrees)")
LANE_OFFSET_GAUGE= Gauge(  "control_lane_offset",            "Latest lane offset (-1 to +1)")
CMD_LATENCY      = Histogram("control_command_latency_sec",  "Command dispatch latency")

# ── Config ────────────────────────────────────────────────────────────────────
MQTT_BROKER  = os.getenv("MQTT_BROKER",    "mosquitto")
MQTT_PORT    = int(os.getenv("MQTT_PORT",  "1883"))
METRICS_PORT = int(os.getenv("METRICS_PORT", "8005"))

# Drive motor speed (% of PWM max, 0-100)
BASE_SPEED   = int(os.getenv("BASE_SPEED",  "55"))
SLOW_SPEED   = int(os.getenv("SLOW_SPEED",  "30"))
TURN_GAIN    = float(os.getenv("TURN_GAIN", "35.0"))   # offset → steering angle scale

# Servo centre and limits (degrees)
SERVO_CENTER = int(os.getenv("SERVO_CENTER", "90"))
SERVO_MAX    = int(os.getenv("SERVO_MAX",    "140"))
SERVO_MIN    = int(os.getenv("SERVO_MIN",    "40"))

# Path recovery parameters
RECOVERY_TIMEOUT  = float(os.getenv("RECOVERY_TIMEOUT",  "3.0"))   # s before recovery activates
RECOVERY_DURATION = float(os.getenv("RECOVERY_DURATION", "1.5"))   # s of recovery manoeuvre

# GPIO pins (BCM)
# L298N
ENA_PIN = int(os.getenv("ENA_PIN", "12"))   # Left / rear-left  (PWM)
IN1_PIN = int(os.getenv("IN1_PIN", "20"))
IN2_PIN = int(os.getenv("IN2_PIN", "21"))
ENB_PIN = int(os.getenv("ENB_PIN", "13"))   # Right / rear-right (PWM)
IN3_PIN = int(os.getenv("IN3_PIN", "19"))
IN4_PIN = int(os.getenv("IN4_PIN", "26"))
# Servo
SERVO_PIN = int(os.getenv("SERVO_PIN", "17"))

TOPIC_LANE     = "cv/lane"
TOPIC_TRAFFIC  = "cv/traffic"
TOPIC_OBSTACLE = "system/obstacle"
TOPIC_STEERING = "control/steering"
TOPIC_HEALTH   = "system/control/health"


# ─────────────────────────────────────────────────────────────────────────────
#  Hardware Drivers
# ─────────────────────────────────────────────────────────────────────────────

class L298NDriver:
    """
    Differential drive controller for L298N H-bridge.
    Provides forward/backward/stop with individual left/right PWM speeds.
    """

    def __init__(self):
        self._gpio = None
        self._pwm_a = None
        self._pwm_b = None
        self._sim   = False

        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            for pin in (ENA_PIN, IN1_PIN, IN2_PIN, ENB_PIN, IN3_PIN, IN4_PIN):
                GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

            self._pwm_a = GPIO.PWM(ENA_PIN, 1000)   # 1 kHz PWM
            self._pwm_b = GPIO.PWM(ENB_PIN, 1000)
            self._pwm_a.start(0)
            self._pwm_b.start(0)
            self._gpio  = GPIO
            logger.info("L298N motor driver ready")
        except (ImportError, RuntimeError):
            self._sim = True
            logger.warning("RPi.GPIO unavailable — motor driver in simulation mode")

    def forward(self, left_pct: float, right_pct: float):
        """Drive both motors forward at given duty cycle (0-100)."""
        if self._sim:
            logger.debug(f"[SIM] FORWARD L={left_pct:.0f}% R={right_pct:.0f}%")
            return
        GPIO = self._gpio
        GPIO.output(IN1_PIN, GPIO.HIGH); GPIO.output(IN2_PIN, GPIO.LOW)
        GPIO.output(IN3_PIN, GPIO.HIGH); GPIO.output(IN4_PIN, GPIO.LOW)
        self._pwm_a.ChangeDutyCycle(min(100, max(0, left_pct)))
        self._pwm_b.ChangeDutyCycle(min(100, max(0, right_pct)))

    def stop(self):
        if self._sim:
            logger.debug("[SIM] STOP")
            return
        GPIO = self._gpio
        GPIO.output(IN1_PIN, GPIO.LOW); GPIO.output(IN2_PIN, GPIO.LOW)
        GPIO.output(IN3_PIN, GPIO.LOW); GPIO.output(IN4_PIN, GPIO.LOW)
        self._pwm_a.ChangeDutyCycle(0)
        self._pwm_b.ChangeDutyCycle(0)

    def cleanup(self):
        self.stop()
        if self._gpio:
            if self._pwm_a: self._pwm_a.stop()
            if self._pwm_b: self._pwm_b.stop()
            self._gpio.cleanup([ENA_PIN, IN1_PIN, IN2_PIN,
                                 ENB_PIN, IN3_PIN, IN4_PIN])


class ServoDriver:
    """
    Ackermann-style steering via a servo motor.
    Angle 90° = straight, <90 = left, >90 = right.
    """

    def __init__(self):
        self._gpio   = None
        self._pwm    = None
        self._sim    = False
        self._angle  = SERVO_CENTER

        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(SERVO_PIN, GPIO.OUT)
            self._pwm  = GPIO.PWM(SERVO_PIN, 50)   # 50 Hz standard servo
            self._pwm.start(self._angle_to_duty(SERVO_CENTER))
            self._gpio = GPIO
            logger.info(f"Servo driver ready → GPIO {SERVO_PIN}")
        except (ImportError, RuntimeError):
            self._sim = True
            logger.warning("RPi.GPIO unavailable — servo in simulation mode")

    @staticmethod
    def _angle_to_duty(angle: float) -> float:
        """Map 0-180° to 2.5-12.5% duty cycle (standard servo)."""
        return 2.5 + (angle / 180.0) * 10.0

    def set_angle(self, angle: float):
        angle = max(SERVO_MIN, min(SERVO_MAX, angle))
        self._angle = angle
        if self._sim:
            logger.debug(f"[SIM] SERVO → {angle:.1f}°")
            return
        self._pwm.ChangeDutyCycle(self._angle_to_duty(angle))

    @property
    def angle(self) -> float:
        return self._angle

    def center(self):
        self.set_angle(SERVO_CENTER)

    def cleanup(self):
        self.center()
        if self._gpio:
            if self._pwm: self._pwm.stop()
            self._gpio.cleanup(SERVO_PIN)


# ─────────────────────────────────────────────────────────────────────────────
#  Control Service
# ─────────────────────────────────────────────────────────────────────────────

class ControlService:
    def __init__(self):
        self.motors  = L298NDriver()
        self.servo   = ServoDriver()
        self._start  = time.time()

        # Latest perception data (thread-safe via lock)
        self._lock           = threading.Lock()
        self._lane_offset    = 0.0
        self._lane_detected  = False
        self._lane_ts        = 0.0
        self._traffic_cmd    = "HOLD"       # "STOP" | "GO" | "HOLD"
        self._obstacle_sev   = "NONE"       # "NONE" | "SLOW" | "STOP"
        self._recovering     = False
        self._recovery_end   = 0.0

        # Local copies to avoid accessing private Prometheus internals (_value.get())
        self._current_speed  = 0.0
        self._current_angle  = float(SERVO_CENTER)
        self._last_health_ts = 0.0

        self.client = mqtt.Client(CallbackAPIVersion.VERSION1, client_id="control_service", clean_session=True)
        self.client.will_set(TOPIC_HEALTH,
                             json.dumps({"status": "offline", "service": "control"}),
                             retain=True)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message

    # ── MQTT ──────────────────────────────────────────────────────────────────
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            logger.info(f"Connected to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
            client.subscribe([(TOPIC_LANE, 0), (TOPIC_TRAFFIC, 0), (TOPIC_OBSTACLE, 0)])
            logger.info(f"Subscribed: {TOPIC_LANE}, {TOPIC_TRAFFIC}, {TOPIC_OBSTACLE}")
        else:
            logger.error(f"MQTT connect failed (rc={rc})")

    def _on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload)
        except Exception:
            return

        with self._lock:
            if msg.topic == TOPIC_LANE:
                self._lane_offset   = data.get("offset", 0.0)
                self._lane_detected = data.get("lane_detected", False)
                self._lane_ts       = data.get("timestamp", time.time())

            elif msg.topic == TOPIC_TRAFFIC:
                self._traffic_cmd = data.get("command", "HOLD")

            elif msg.topic == TOPIC_OBSTACLE:
                self._obstacle_sev = data.get("severity", "NONE")

    # ── Decision loop ──────────────────────────────────────────────────────────
    def _offset_to_angle(self, offset: float) -> float:
        """
        Map normalized lane offset [-1, +1] to servo angle.
        Positive offset (car to the right) → steer left (angle < 90).
        Negative offset (car to the left)  → steer right (angle > 90).
        """
        angle = SERVO_CENTER - (offset * TURN_GAIN)
        return max(SERVO_MIN, min(SERVO_MAX, angle))

    def _decide_and_drive(self):
        with self._lock:
            traffic_cmd  = self._traffic_cmd
            obstacle_sev = self._obstacle_sev
            offset       = self._lane_offset
            lane_ok      = self._lane_detected
            lane_ts      = self._lane_ts

        now = time.time()

        # ── Priority 1: Traffic light ─────────────────────────────────────
        if traffic_cmd == "STOP":
            self.motors.stop()
            self.servo.center()
            STOPS_TOTAL.inc()
            SPEED_GAUGE.set(0)
            STEERING_GAUGE.set(SERVO_CENTER)
            self._publish_cmd("TRAFFIC_STOP", 0, SERVO_CENTER)
            return

        # ── Priority 2: Obstacle ──────────────────────────────────────────
        if obstacle_sev == "STOP":
            self.motors.stop()
            self.servo.center()
            STOPS_TOTAL.inc()
            SPEED_GAUGE.set(0)
            STEERING_GAUGE.set(SERVO_CENTER)
            self._publish_cmd("OBSTACLE_STOP", 0, SERVO_CENTER)
            return

        target_speed = SLOW_SPEED if obstacle_sev == "SLOW" else BASE_SPEED

        # ── Priority 3: Path recovery (lane lost too long) ────────────────
        if not lane_ok and lane_ts > 0 and (now - lane_ts) > RECOVERY_TIMEOUT:
            if not self._recovering:
                self._recovering   = True
                self._recovery_end = now + RECOVERY_DURATION
                RECOVERY_EVENTS.inc()
                logger.warning("Lane lost — activating path recovery")

        if self._recovering:
            if now < self._recovery_end:
                # Slow down and steer back to centre
                self.motors.forward(target_speed * 0.5, target_speed * 0.5)
                self.servo.center()
                SPEED_GAUGE.set(target_speed * 0.5)
                STEERING_GAUGE.set(SERVO_CENTER)
                self._publish_cmd("RECOVERY", target_speed * 0.5, SERVO_CENTER)
                return
            else:
                self._recovering = False
                logger.info("Path recovery complete — resuming normal operation")

        # ── Priority 4: Lane following ────────────────────────────────────
        if not lane_ok:
            # No lane data yet / brief gap — keep going straight
            self.motors.forward(target_speed, target_speed)
            self.servo.center()
            SPEED_GAUGE.set(target_speed)
            STEERING_GAUGE.set(SERVO_CENTER)
            self._publish_cmd("STRAIGHT_NO_LANE", target_speed, SERVO_CENTER)
            return

        # Steering angle from lane offset
        angle = self._offset_to_angle(offset)

        # Differential speed: slow the inside wheel during turns
        turn_factor  = abs(offset)                              # 0 = straight, 1 = max turn
        inner_speed  = target_speed * (1 - turn_factor * 0.5)
        outer_speed  = target_speed

        if offset > 0:    # Lane centre is right → steer left
            self.motors.forward(inner_speed, outer_speed)
        else:             # Lane centre is left → steer right
            self.motors.forward(outer_speed, inner_speed)

        self.servo.set_angle(angle)
        SPEED_GAUGE.set(target_speed)
        STEERING_GAUGE.set(angle)
        LANE_OFFSET_GAUGE.set(offset)
        COMMANDS_SENT.inc()
        self._publish_cmd("LANE_FOLLOW", target_speed, angle)

    def _publish_cmd(self, action: str, speed: float, angle: float):
        self._current_speed = speed
        self._current_angle = angle
        payload = {
            "action":    action,
            "speed":     round(speed, 1),
            "angle":     round(angle, 1),
            "timestamp": time.time(),
        }
        self.client.publish(TOPIC_STEERING, json.dumps(payload), qos=0)

    # ── Service lifecycle ──────────────────────────────────────────────────────
    def run(self):
        logger.info("═══ Control Service starting ═══")
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
        logger.info("Control loop running at ~20 Hz")

        try:
            while True:
                t0 = time.time()
                self._decide_and_drive()
                CMD_LATENCY.observe(time.time() - t0)

                # Health heartbeat throttled to every 5 s (not every 20 Hz loop tick)
                if time.time() - self._last_health_ts > 5:
                    self.client.publish(TOPIC_HEALTH, json.dumps({
                        "status":   "online",
                        "service":  "control",
                        "speed":    round(self._current_speed, 1),
                        "angle":    round(self._current_angle, 1),
                        "uptime_s": round(time.time() - self._start),
                    }), retain=True)
                    self._last_health_ts = time.time()

                time.sleep(0.05)   # 20 Hz control loop

        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        logger.info("Stopping Control Service…")
        self.motors.stop()
        self.servo.center()
        self.motors.cleanup()
        self.servo.cleanup()
        self.client.publish(TOPIC_HEALTH,
                            json.dumps({"status": "offline", "service": "control"}),
                            retain=True)
        self.client.loop_stop()
        self.client.disconnect()
        logger.info("Control Service stopped ✓")


if __name__ == "__main__":
    svc = ControlService()
    try:
        svc.run()
    except KeyboardInterrupt:
        svc.stop()

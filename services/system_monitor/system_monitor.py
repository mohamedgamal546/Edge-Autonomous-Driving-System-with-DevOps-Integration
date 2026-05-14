"""
System Monitor Service
=======================
Aggregates health, status, and metrics from all other services.
Acts as a single observability hub — publishes a consolidated
system status to MQTT and exposes it to Prometheus/Grafana.

MQTT Topics Subscribed:
    system/camera/health
    system/lane/health
    system/traffic/health
    system/obstacle/health
    system/control/health

MQTT Topics Published:
    system/status  → consolidated system health JSON
"""

import time
import logging
import json
import os
import threading
import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion
from prometheus_client import start_http_server, Gauge, Info

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [MONITOR] %(levelname)s - %(message)s"
)
logger = logging.getLogger("system_monitor")

# ── Prometheus Metrics ────────────────────────────────────────────────────────
SERVICE_UP = {
    svc: Gauge(f"service_up_{svc}", f"1 if {svc} service is online")
    for svc in ("camera", "lane", "traffic", "obstacle", "control")
}
SYSTEM_HEALTHY = Gauge("system_healthy", "1 if ALL services are online")
BUILD_INFO     = Info("autonomous_car_build", "Build metadata")

# ── Config ────────────────────────────────────────────────────────────────────
MQTT_BROKER   = os.getenv("MQTT_BROKER",   "mosquitto")
MQTT_PORT     = int(os.getenv("MQTT_PORT", "1883"))
METRICS_PORT  = int(os.getenv("METRICS_PORT", "8006"))
STALE_TIMEOUT = float(os.getenv("STALE_TIMEOUT", "15.0"))   # Mark service dead after N s silence

HEALTH_TOPICS = {
    "camera":   "system/camera/health",
    "lane":     "system/lane/health",
    "traffic":  "system/traffic/health",
    "obstacle": "system/obstacle/health",
    "control":  "system/control/health",
}
TOPIC_STATUS  = "system/status"


class SystemMonitor:
    def __init__(self):
        self._services: dict[str, dict] = {
            svc: {"status": "unknown", "last_seen": 0.0, "data": {}}
            for svc in HEALTH_TOPICS
        }
        self._lock       = threading.Lock()
        self._start_time = time.time()

        BUILD_INFO.info({
            "project":  "Edge Autonomous Driving System",
            "version":  os.getenv("APP_VERSION", "1.0.0"),
            "platform": "Raspberry Pi 4",
            "author":   os.getenv("AUTHOR", "graduation-project"),
        })

        self.client = mqtt.Client(CallbackAPIVersion.VERSION1, client_id="system_monitor", clean_session=True)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_health_message

    # ── MQTT ──────────────────────────────────────────────────────────────────
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            logger.info(f"Connected to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
            topics = [(t, 0) for t in HEALTH_TOPICS.values()]
            client.subscribe(topics)
            logger.info(f"Subscribed to {len(topics)} health topics")
        else:
            logger.error(f"MQTT connect failed (rc={rc})")

    def _on_health_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload)
        except Exception:
            return

        # Identify which service sent this
        svc_name = next(
            (name for name, topic in HEALTH_TOPICS.items() if topic == msg.topic),
            None
        )
        if not svc_name:
            return

        with self._lock:
            self._services[svc_name] = {
                "status":    data.get("status", "unknown"),
                "last_seen": time.time(),
                "data":      data,
            }

        is_up = data.get("status") == "online"
        SERVICE_UP[svc_name].set(1 if is_up else 0)
        logger.debug(f"Health update [{svc_name}]: {data.get('status')}")

    # ── Status computation ─────────────────────────────────────────────────────
    def _compute_status(self) -> dict:
        now = time.time()
        summary = {}
        all_ok  = True

        with self._lock:
            for svc, info in self._services.items():
                # Mark as offline if we haven't heard from it recently
                age = now - info["last_seen"] if info["last_seen"] > 0 else 999
                if age > STALE_TIMEOUT and info["status"] != "offline":
                    info["status"] = "stale"
                    SERVICE_UP[svc].set(0)

                is_ok = info["status"] == "online"
                if not is_ok:
                    all_ok = False

                summary[svc] = {
                    "status":   info["status"],
                    "age_s":    round(age, 1),
                    "data":     info["data"],
                }

        SYSTEM_HEALTHY.set(1 if all_ok else 0)

        return {
            "system_healthy": all_ok,
            "services":       summary,
            "uptime_s":       round(now - self._start_time),
            "timestamp":      now,
        }

    # ── Run ───────────────────────────────────────────────────────────────────
    def run(self):
        logger.info("═══ System Monitor starting ═══")
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
        logger.info("Monitoring loop active (publish every 3 s)")

        try:
            while True:
                status = self._compute_status()
                self.client.publish(TOPIC_STATUS, json.dumps(status), retain=True)

                # Console summary
                healthy = "✓ ALL OK" if status["system_healthy"] else "✗ DEGRADED"
                states  = "  ".join(
                    f"{s}={'↑' if d['status']=='online' else '↓'}"
                    for s, d in status["services"].items()
                )
                logger.info(f"[{healthy}]  {states}  uptime={status['uptime_s']}s")

                time.sleep(3)
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        logger.info("Stopping System Monitor…")
        self.client.loop_stop()
        self.client.disconnect()
        logger.info("System Monitor stopped ✓")


if __name__ == "__main__":
    mon = SystemMonitor()
    try:
        mon.run()
    except KeyboardInterrupt:
        mon.stop()

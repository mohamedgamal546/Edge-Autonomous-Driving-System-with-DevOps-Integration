# 🚗 Edge Autonomous Driving System
> A fully containerised, microservices-based autonomous vehicle built on Raspberry Pi 4 with end-to-end DevOps integration.

---

## 📌 Overview

This system enables a physical car to drive autonomously by combining real-time computer vision, sensor fusion, and hardware motor control — all running **on the edge** (Raspberry Pi 4) with zero cloud dependency.

Every perception algorithm runs as an isolated Python microservice that communicates asynchronously via **Mosquitto MQTT**. The entire stack is containerised with **Docker**, deployed with **Ansible**, monitored with **Prometheus + Grafana**, and continuously delivered via **GitHub Actions**.

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Raspberry Pi 4                               │
│                                                                     │
│  ┌──────────┐    ┌──────────────┐    ┌─────────────────────────┐   │
│  │  Pi Cam  │───▶│    Camera    │───▶│                         │   │
│  └──────────┘    │   Service    │    │                         │   │
│                  └──────────────┘    │   Mosquitto MQTT Broker │   │
│  ┌──────────┐    ┌──────────────┐    │   ─────────────────────  │   │
│  │ HC-SR04  │───▶│  Obstacle    │───▶│  cv/camera/frame        │   │
│  │Ultrasonic│    │  Detection   │    │  cv/lane                │   │
│  └──────────┘    └──────────────┘    │  cv/traffic             │   │
│                  ┌──────────────┐    │  system/obstacle        │   │
│                  │     Lane     │◀───│  control/steering       │   │
│                  │  Detection   │───▶│  system/*/health        │   │
│                  └──────────────┘    │                         │   │
│                  ┌──────────────┐    └─────────────────────────┘   │
│                  │   Traffic    │◀───────────────┘                  │
│                  │    Light     │                                   │
│                  └──────────────┘                                   │
│                  ┌──────────────┐    ┌──────────────────────────┐  │
│                  │   Control    │───▶│  L298N Motor Driver      │  │
│                  │   Service    │───▶│  Servo (Ackermann)       │  │
│                  └──────────────┘    └──────────────────────────┘  │
│                  ┌──────────────┐    ┌──────────────────────────┐  │
│                  │   System     │───▶│  Prometheus + Grafana    │  │
│                  │   Monitor    │    │  (Port 3000)             │  │
│                  └──────────────┘    └──────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 🧠 Microservices

| Service | MQTT Publish | MQTT Subscribe | Description |
|---|---|---|---|
| **camera** | `cv/camera/frame` | — | Captures frames, manages IR LED for low-light |
| **lane_detection** | `cv/lane` | `cv/camera/frame` | Canny + Hough Lines → lateral offset |
| **obstacle_detection** | `system/obstacle` | `cv/camera/frame` | HC-SR04 ultrasonic + HSV green object detection |
| **traffic_light** | `cv/traffic` | `cv/camera/frame` | HSV red/green blob detection with confirmation window |
| **control** | `control/steering` | `cv/lane`, `cv/traffic`, `system/obstacle` | Fuses all inputs → L298N + Servo commands |
| **system_monitor** | `system/status` | `system/*/health` | Aggregates health from all services |

---

## 🔧 Hardware

| Component | Role |
|---|---|
| Raspberry Pi 4 (4 GB) | Central compute — runs all services |
| Raspberry Pi Camera Module | Primary vision sensor |
| HC-SR04 Ultrasonic Sensor | Short-range obstacle detection (mounted under camera) |
| L298N H-Bridge Motor Driver | Rear-wheel drive (differential speed) |
| Servo Motor (SG90) | Front Ackermann steering |
| White LED (GPIO 18) | Automatic low-light assistance |

### GPIO Pinout (BCM)

```
GPIO 12 → L298N ENA (Left Motor PWM)    GPIO 18 → LED
GPIO 13 → L298N ENB (Right Motor PWM)   GPIO 17 → Servo PWM
GPIO 20 → IN1    GPIO 21 → IN2          GPIO 23 → HC-SR04 TRIG
GPIO 19 → IN3    GPIO 26 → IN4          GPIO 24 → HC-SR04 ECHO
```

---

## 🤖 Computer Vision Pipeline

### Lane Detection (Canny + Hough)
```
BGR Frame → Grayscale → GaussianBlur(5,5)
         → Canny(50, 150)
         → Trapezoidal ROI Mask
         → HoughLinesP(threshold=40, minLen=80, maxGap=25)
         → Line averaging (left/right separation by slope)
         → Normalized offset [-1, +1]
         → Exponential smoothing (α=0.3)
```

### Traffic Light Detection
- HSV colour segmentation (dual-range for red wrap-around)
- N-frame confirmation window to reject false positives
- ROI: upper-centre 30% of frame

### Obstacle Detection
- **Visual**: HSV green masking → contour area threshold → MQTT `severity`
- **Ultrasonic**: HC-SR04 polling at 20 Hz → median of 3 readings
- Combined: `NONE / SLOW / STOP` severity levels

### Control Priority (highest → lowest)
```
1. Traffic STOP      → halt, servo centre
2. Obstacle STOP     → halt, servo centre
3. Obstacle SLOW     → 55% speed, continue steering
4. Path Recovery     → slow + centre after 3 s lane loss
5. Lane Following    → differential speed + Ackermann steering
```

---

## 🚀 Getting Started

### Prerequisites
- Raspberry Pi 4 (Raspberry Pi OS Bookworm 64-bit)
- Docker + Docker Compose v2
- Python 3.11+

### Quick Start

```bash
# Clone the repository
git clone https://github.com/YOUR_USERNAME/autonomous-car.git
cd autonomous-car

# Start the full stack
docker compose up -d

# Watch logs
docker compose logs -f

# Open Grafana dashboard
# → http://<raspberry-pi-ip>:3000
# → Login: admin / autonomouscar

# Open Prometheus
# → http://<raspberry-pi-ip>:9090
```

### Development (without Pi hardware)
All services run in **simulation mode** automatically when `RPi.GPIO` is not available — perfect for developing on a laptop.

```bash
pip install -r requirements.txt

# Run any service standalone
python services/lane_detection/lane_detection_service.py
python services/camera/camera_service.py
```

### Run Tests

```bash
pip install pytest pytest-cov
pytest tests/ -v --cov=services
```

---

## 🔄 CI/CD Pipeline (GitHub Actions)

```
Push to main/develop
        │
        ▼
  ┌─────────┐
  │  Lint   │  flake8 + black + isort
  └────┬────┘
       │
       ▼
  ┌─────────┐
  │  Tests  │  pytest + coverage report
  └────┬────┘
       │
       ▼
  ┌─────────────────────────────────────┐
  │  Build (matrix: 6 services)         │
  │  linux/amd64 + linux/arm64 (Pi)     │
  │  Push → GitHub Container Registry   │
  └────┬────────────────────────────────┘
       │  (main branch only)
       ▼
  ┌─────────┐
  │ Deploy  │  Ansible → Raspberry Pi via SSH
  └─────────┘
```

### Required GitHub Secrets

| Secret | Description |
|---|---|
| `PI_HOST` | Raspberry Pi IP address |
| `PI_USER` | SSH username on Pi |
| `PI_SSH_KEY` | Private SSH key for Pi access |

---

## 📊 Monitoring

### Prometheus Metrics (per service)

| Metric | Description |
|---|---|
| `camera_fps_current` | Live capture FPS |
| `lane_offset_normalized` | Lateral position (-1 to +1) |
| `obstacle_distance_cm` | Ultrasonic range |
| `traffic_light_state` | 0=none, 1=red, 2=green |
| `control_steering_angle` | Servo angle (40–140°) |
| `control_speed_pwm` | Drive motor duty cycle |
| `service_up_*` | Per-service health (0/1) |
| `system_healthy` | All services OK (0/1) |

### Grafana Dashboard
Pre-built dashboard at `monitoring/grafana/dashboards/system_overview.json` — auto-provisioned on startup.

Panels include:
- System health status (all-green / degraded)
- Lane offset timeline
- Detection event rates
- Frame processing latency (p99)
- Service up/down status

---

## 📁 Project Structure

```
autonomous-car/
├── services/
│   ├── camera/                 # Frame capture + LED control
│   ├── lane_detection/         # Canny + Hough lane following
│   ├── obstacle_detection/     # Ultrasonic + visual obstacle detection
│   ├── traffic_light/          # HSV traffic light detection
│   ├── control/                # L298N + Servo motor control
│   └── system_monitor/         # Health aggregation
├── devops/
│   ├── docker/
│   │   ├── Dockerfile.service  # Multi-stage, multi-service image
│   │   └── mosquitto.conf      # MQTT broker config
│   └── ansible/
│       ├── deploy.yml          # Main deployment playbook
│       ├── inventory/          # Pi host inventory
│       └── roles/              # App + monitoring roles
├── monitoring/
│   ├── prometheus/
│   │   └── prometheus.yml      # Scrape config (all 6 services)
│   └── grafana/
│       ├── dashboards/         # Pre-built JSON dashboards
│       └── provisioning/       # Auto-provision on startup
├── tests/
│   ├── test_lane_detection.py
│   └── test_traffic_light.py
├── .github/workflows/
│   └── ci_cd.yml               # Full CI/CD pipeline
├── docker-compose.yml           # Full stack definition
└── requirements.txt
```

---

## 🛠️ Configuration

All parameters are controlled via environment variables — no code changes needed.

```bash
# Example: tune lane detection sensitivity
CANNY_LOW=40 CANNY_HIGH=120 HOUGH_MIN_LEN=60 docker compose up lane_detection

# Example: adjust control speeds
BASE_SPEED=45 SLOW_SPEED=25 TURN_GAIN=30 docker compose up control
```

See each service file header for the full list of supported environment variables.

---

## 📄 License

MIT License — see [LICENSE](LICENSE)

---

## 👤 Author

Mohamed Gamal Nasser  

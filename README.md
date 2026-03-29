# VelocityOS —  v1.0
### *A Production-Grade Embedded Operating System for ESP32-S3 Robotic Platforms*

> **Powered by Aosmic Studio × Absolute Tech**

---

## Overview

VelocityOS is an ultra-advanced embedded OS built for the ESP32-S3, designed around a **4WD modular robotic platform**. It implements a hybrid architecture combining:

- **Microkernel principles** — minimal, isolated kernel with services in user-space tasks
- **Service-Oriented Architecture (SOA)** — all capabilities as independent, message-driven services
- **Event-Driven Reactive System** — non-blocking pub/sub message bus at the center of all communication

---

## Architecture Diagram

```
╔══════════════════════════════════════════════════════════════════╗
║                                                                               ║
║                       VelocityOS v1.0                            ║
╠══════════════════════════════════════════════════════════════════╣
║                                                                  ║
║  ┌──────────────────── APPLICATION LAYER ─────────────────────┐ ║
║  │  Web Dashboard   │  REST API   │  WebSocket  │  BT Gamepad │ ║
║  └─────────────────────────┬───────────────────────────────────┘ ║
║                             │                                    ║
║  ┌──────────────── SERVICE ORCHESTRATION LAYER ───────────────┐ ║
║  │  Motion Svc │ Sensor Svc │ AI Engine │ Power Svc │ OTA Svc │ ║
║  │  Comm Svc   │ Vision Svc │ Security  │ Telemetry │ ...      │ ║
║  └──────────────────────────┬──────────────────────────────────┘ ║
║                              │                                   ║
║  ╔══════════════ CENTRAL MESSAGE BUS ═══════════════╗           ║
║  ║  /motor/cmd  /sensor/data  /ai/decision           ║           ║
║  ║  /system/alert  /estop  /telemetry  /power/status ║           ║
║  ║  Non-blocking · Queue-based · Fan-out delivery    ║           ║
║  ╚═══════════════════════════════════════════════════╝           ║
║                              │                                   ║
║  ┌─────────────── HYPERKERNEL LAYER ──────────────────────────┐ ║
║  │  Task Manager │ IPC System │ Service Registry │ Scheduler  │ ║
║  │  Module Registry │ ESTOP Handler │ Telemetry Engine        │ ║
║  └──────────────────────────┬──────────────────────────────────┘ ║
║                              │                                   ║
║  ┌─────────────── HARDWARE ABSTRACTION LAYER ─────────────────┐ ║
║  │  GPIO HAL │ PWM HAL │ I2C HAL │ SPI HAL │ ADC HAL         │ ║
║  └──────────────────────────┬──────────────────────────────────┘ ║
║                              │                                   ║
║  ┌─────────────── HARDWARE (ESP32-S3) ────────────────────────┐ ║
║  │  Motors × 4  │ Ultrasonic × 3 │ Gas Sensor │ Flame Sensor  │ ║
║  │  Water Pump  │ IMU (I2C)      │ Camera     │ Battery ADC   │ ║
║  └──────────────────────────────────────────────────────────────┘ ║
╚══════════════════════════════════════════════════════════════════╝
```

---

## Module System

```
MODULE REGISTRY
├── MotorDriver-TB6612    [DRIVE]   2000mW  ✓ active
├── HC-SR04-Array         [SENSOR]    45mW  ✓ active
├── MQ2-GasSensor         [SENSOR]   900mW  ✓ active
├── WaterPump-5V          [ACTUATOR] 3000mW  ○ on-demand
└── FlameSensor-IR        [SENSOR]    15mW  ✓ active
```

---

## Message Bus Topics

| Topic              | Publisher         | Subscribers                  |
|--------------------|-------------------|------------------------------|
| `/motor/cmd`       | CommSvc, AI       | MotionSvc                    |
| `/motor/status`    | MotionSvc         | Telemetry, Dashboard         |
| `/sensor/data`     | SensorSvc         | AI Engine, Telemetry         |
| `/sensor/alert`    | SensorSvc         | AI Engine, System            |
| `/system/alert`    | Any               | CommSvc, Dashboard           |
| `/ai/decision`     | AI Engine         | MotionSvc, Telemetry         |
| `/power/status`    | PowerSvc          | AI Engine, Telemetry         |
| `/telemetry`       | TelemetrySvc      | WebSocket Broadcaster        |
| `/obstacle`        | SensorSvc         | MotionSvc (safety override)  |
| `/estop`           | Any (kernel)      | MotionSvc (hard stop)        |
| `/ota/status`      | OTA Manager       | Dashboard                    |
| `/module/registry` | Kernel            | Dashboard                    |

---

## AI Decision Engine — Rule Table

```
PRIORITY  CONDITION                           ACTION
────────  ──────────────────────────────────  ──────────────────────────────────
1         manual_override == true             → STATE: MANUAL (pass-through)
2         estop_active == true                → STATE: EMERGENCY, halt all motion
3         gas_level > 70%                     → STATE: GAS_RESPONSE, stop + pump
4         flame_detected OR smoke_detected    → STATE: FIRE_RESPONSE, retreat + pump
5         battery_voltage < 6.5V             → STATE: LOW_BATTERY, stop + alert
6         obstacle_front < 20cm              → STATE: AVOID, turn (smart direction)
7         obstacle_front < 40cm              → STATE: PATROL (slow)
8         idle_time > 5s                     → STATE: PATROL, auto-navigate
DEFAULT                                       → STATE: IDLE
```

---

## Folder Structure

```
VelocityOS/
├── main.cpp                    ← OS boot entry point
├── kernel/
│   ├── hyperkernel.h           ← Kernel API + types
│   └── hyperkernel.c           ← Kernel implementation
├── hal/
│   ├── hal.h                   ← Hardware abstraction API
│   └── hal.c                   ← GPIO / PWM / I2C / ADC impl
├── bus/
│   ├── message_bus.h           ← Pub/Sub message bus API
│   └── message_bus.c           ← Bus dispatcher + fan-out
├── services/
│   ├── motion_service.h/.c     ← Differential drive + safety
│   ├── sensor_service.c        ← Sensor aggregation
│   ├── power_service.c         ← Power intelligence
│   └── telemetry_service.c     ← Real-time telemetry engine
├── ai/
│   ├── ai_engine.h             ← AI state machine API
│   └── ai_engine.c             ← Rule-based decision engine
├── modules/
│   └── modules.c               ← Hot-plug module registry
├── webserver/
│   └── comm_service.c          ← WiFi + REST + WebSocket
├── security/
│   ├── security.h              ← Auth API
│   └── security.c              ← Token auth + fail-safe
├── ota/
│   └── ota_manager.c           ← Dual-partition OTA
└── ui/
    └── dashboard.html          ← Next-gen web dashboard
```

---

## Task Map & Priorities

| Task             | Priority | Core | Stack  | Rate  |
|------------------|----------|------|--------|-------|
| HyperKernel Mon  | 24       | 0    | 8 KB   | 5s    |
| Bus Dispatcher   | 23       | 0    | 4 KB   | event |
| Motion Control   | 22       | 0    | 6 KB   | 50 Hz |
| Sensor Polling   | 21       | 1    | 4 KB   | 10 Hz |
| AI Decision      | 20       | 1    | 8 KB   | 20 Hz |
| Communication    | 19       | 1    | 16 KB  | event |
| Power Monitor    | 17       | 1    | 3 KB   | 1 Hz  |
| Telemetry        | 16       | 1    | 4 KB   | 20 Hz |
| WS Broadcast     | 19       | 1    | 4 KB   | event |

---

## REST API Reference

| Method | Endpoint         | Description             |
|--------|-----------------|-------------------------|
| GET    | `/api/status`   | System + kernel status  |
| POST   | `/api/auth`     | Authenticate, get token |
| POST   | `/api/move`     | Send motion command     |
| POST   | `/api/estop`    | Emergency stop          |
| GET    | `/api/modules`  | List all modules        |
| POST   | `/api/ota`      | Trigger OTA update      |
| WS     | `/ws`           | Real-time telemetry     |

---

## Build & Flash (ESP-IDF)

```bash
# Prerequisites
idf.py --version   # requires v5.2+

# Configure
idf.py menuconfig
# Set: Component config → VelocityOS
#   - WIFI_SSID / WIFI_PASSWORD
#   - Admin token

# Build
idf.py build

# Flash
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## Web Dashboard

Open `ui/dashboard.html` in any modern browser.

- **Demo Mode**: Leave IP blank → instant simulated telemetry
- **Live Mode**: Enter robot IP → full real-time control
- **Features**: 3D robot map, live graphs, virtual joystick, module manager, AI state, OTA panel

---

## Security

- Token-based session auth (32-char random tokens)
- Session expiry: 1 hour
- Max failed attempts: 5 → 1-minute lockout
- All commands validated before execution
- ESTOP requires no auth (always accessible)

---

## License

Apache 2.0 — Aosmic Studio × Absolute Tech

---

*Powered by Aosmic Studio × Absolute Tech*
e Tech*

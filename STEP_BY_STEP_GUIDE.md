# VelocityOS — Complete Step-by-Step Build Guide
### Aosmic HyperKernel OS v1.0 | ESP32-S3 4WD Robot
> Apache License 2.0 — Aosmic Studio × Absolute Tech

---

## PART 1 — HARDWARE ASSEMBLY

### Components Required

| Component | Qty | Notes |
|---|---|---|
| ESP32-S3 DevKitC-1 | 1 | N16R8 variant recommended |
| L298N or TB6612FNG Motor Driver | 2 | TB6612 preferred (less heat) |
| DC Gear Motors (6V–12V) | 4 | ~200 RPM recommended |
| 4WD Robot Chassis | 1 | 200mm × 150mm or larger |
| HC-SR04 Ultrasonic Sensor | 3 | Front, rear, left |
| MQ-2 Gas Sensor Module | 1 | 5V supply |
| Flame Sensor IR Module | 1 | Digital output |
| Water Pump + Relay Module | 1 | 5V relay |
| 2S LiPo Battery 7.4V 3000mAh | 1 | XT60 connector |
| LiPo Battery Management System | 1 | 2S BMS |
| Voltage Divider (100kΩ + 47kΩ) | 1 | For battery ADC |
| Buzzer (active) | 1 | 3.3V compatible |
| LEDs (green + red) | 2 | With 220Ω resistors |
| Jumper wires | 30+ | Male–female and male–male |
| USB-C cable | 1 | For flashing |

---

### Step 1 — Assemble the Chassis

```
1. Mount four motors into the 4WD chassis frame.
2. Attach wheels to motor shafts.
3. Mount ESP32-S3 DevKit on chassis using M3 standoffs.
4. Mount both motor driver boards on the chassis.
5. Secure battery holder/tray at the center of mass.
```

---

### Step 2 — Wire the Motor Drivers

**Motor Driver 1 (Front Left + Front Right):**
```
TB6612FNG          ESP32-S3
PWMA       ──────► GPIO 1   (Front Left PWM)
AIN1       ──────► GPIO 2   (Front Left DIR A)
AIN2       ──────► GPIO 3   (Front Left DIR B)
PWMB       ──────► GPIO 4   (Front Right PWM)
BIN1       ──────► GPIO 5   (Front Right DIR A)
BIN2       ──────► GPIO 6   (Front Right DIR B)
VCC        ──────► 3.3V
GND        ──────► GND
STBY       ──────► 3.3V (always enabled)
VM         ──────► Battery + (7.4V)
```

**Motor Driver 2 (Rear Left + Rear Right):**
```
PWMA       ──────► GPIO 7   (Rear Left PWM)
AIN1       ──────► GPIO 8
AIN2       ──────► GPIO 9
PWMB       ──────► GPIO 10  (Rear Right PWM)
BIN1       ──────► GPIO 11
BIN2       ──────► GPIO 12
VM         ──────► Battery + (7.4V)
GND        ──────► GND
```

---

### Step 3 — Wire the Sensors

**Ultrasonic Sensors (HC-SR04):**
```
Front Ultrasonic:
  TRIG ──► GPIO 13    ECHO ──► GPIO 14
  VCC  ──► 5V         GND  ──► GND

Rear Ultrasonic:
  TRIG ──► GPIO 15    ECHO ──► GPIO 16

Left Ultrasonic:
  TRIG ──► GPIO 17    ECHO ──► GPIO 18

⚠ HC-SR04 ECHO outputs 5V — use a voltage divider
  (1kΩ + 2kΩ) to bring it to 3.3V for ESP32.
```

**Gas + Fire Sensors:**
```
MQ-2 Gas Sensor:
  AOUT ──► GPIO 1 (ADC_CH0, via 10kΩ series resistor)
  VCC  ──► 5V
  GND  ──► GND

Flame Sensor IR (digital):
  OUT  ──► GPIO 19 (active LOW)
  VCC  ──► 3.3V
  GND  ──► GND

Smoke Sensor:
  OUT  ──► GPIO 20
```

**Actuators:**
```
Water Pump Relay:
  IN   ──► GPIO 21
  VCC  ──► 5V
  GND  ──► GND

Buzzer (active):
  +    ──► GPIO 22 (through 100Ω resistor)
  -    ──► GND

Status LED (green) + 220Ω:
  +    ──► GPIO 23

Warning LED (red) + 220Ω:
  +    ──► GPIO 24
```

**Battery Monitor (voltage divider):**
```
Battery+ ──[100kΩ]──┬──[47kΩ]── GND
                    └──► GPIO (ADC_CH4)

Measured voltage = ADC_voltage × (100k+47k)/47k = ADC × 3.128
```

**I2C (IMU MPU-6050):**
```
SDA ──► GPIO 38    SCL ──► GPIO 39
VCC ──► 3.3V       GND ──► GND
```

---

### Step 4 — Power System

```
                    ┌────────────────────────┐
  2S LiPo 7.4V ───► │ BMS (2S protection)    │
                    └────────┬───────────────┘
                             │ 7.4V
              ┌──────────────┼───────────────┐
              │              │               │
         [Motor Drivers]  [5V Buck]     [Battery ADC divider]
              │           converter
              │              │ 5V
              │        ┌─────┴──────┐
              │        │  ESP32-S3  │── USB (flash only)
              │        │  3.3V reg  │
              │        └────────────┘
```

---

## PART 2 — FIRMWARE SETUP

### Step 5 — Install ESP-IDF

```bash
# Linux / macOS
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32s3
source export.sh

# Windows — use the ESP-IDF Windows Installer:
# https://dl.espressif.com/dl/esp-idf/

# Verify installation
idf.py --version   # Should print v5.2 or later
```

---

### Step 6 — Clone and Configure VelocityOS

```bash
# Clone the project
git clone https://github.com/your-org/VelocityOS
cd VelocityOS

# Open configuration menu
idf.py menuconfig
```

**In menuconfig, navigate to:**
```
Component config → VelocityOS Configuration →
  ├── WiFi SSID         → Enter your network name
  ├── WiFi Password     → Enter your password
  ├── Admin Token       → Choose a secure token (this is your dashboard password)
  ├── Battery Capacity  → Set to your actual mAh (default 3000)
  └── ESTOP Distance    → Obstacle stop distance in cm (default 20)
```

Press `S` to save, `Q` to quit.

---

### Step 7 — Build the Firmware

```bash
# Build (takes 2–5 minutes first time)
idf.py build

# You should see:
# Project build complete. Binary size: ~1.2MB
# To flash, run: idf.py flash
```

---

### Step 8 — Flash the ESP32-S3

```bash
# Connect ESP32-S3 via USB-C
# Find your port:
ls /dev/tty*          # Linux/Mac — look for ttyUSB0 or ttyACM0
# Device Manager      # Windows — look for COM port

# Flash and monitor
idf.py -p /dev/ttyUSB0 flash monitor
# Use -p COM3 on Windows

# You should see the boot banner:
# ╔═══════════════════════════╗
# ║      VelocityOS           ║
# ║  Aosmic HyperKernel v1.0  ║
# ╚═══════════════════════════╝
# WiFi connected! IP: 192.168.1.XXX
```

Note the IP address — you'll need it for the dashboard.

---

## PART 3 — WEB DASHBOARD

### Step 9 — Open the Dashboard

```
Option A — Direct file (no server needed):
  Open  ui/dashboard.html  in Chrome, Firefox, or Safari

Option B — Serve via Python (optional):
  cd VelocityOS/ui
  python3 -m http.server 8080
  Open http://localhost:8080/dashboard.html

Option C — Demo Mode (no robot required):
  Open dashboard.html → Click "Demo Mode"
  All telemetry will be simulated.
```

---

### Step 10 — Connect to Your Robot

```
1. Open dashboard.html in browser
2. Enter robot IP address (from serial monitor)
3. Enter your Admin Token (set in menuconfig)
4. Click "Connect"
5. All tabs become live with real-time telemetry
```

---

## PART 4 — FIRST RUN & TESTING

### Step 11 — Test Basic Motion

```
In the Control tab:
1. Confirm "Live" appears in the header (green dot)
2. Press and hold ↑ arrow — robot should move forward
3. Test ←, →, ↓ arrows
4. Test the virtual joystick (click and drag)
5. Adjust speed slider (default 60%)
6. Press E-Stop — all motors should halt immediately
```

---

### Step 12 — Run Your First Area Scan

```
In the Area Scan tab:
1. Place robot in the center of the area to scan
2. Ensure at least 3–4m of clear space around
3. Click "Start Scan"
4. Watch the occupancy map fill in real time:
   - Green cells = free space
   - Red cells = obstacles
   - Robot position (amber dot) moves on the map
5. The scan takes 30–90 seconds depending on area size
6. On completion:
   - Blue patrol route appears on the map
   - AI Training Result table populates
   - AI patrol speed is automatically calibrated
7. Click "AI: OFF" in the Control tab to switch to autonomous mode
8. Robot will now patrol the scanned route
```

---

### Step 13 — Verify Sensor Alerts

```
In the Sensors tab, verify:
- Front distance reads correctly (wave hand in front)
- Gas level responds (briefly hold lighter near MQ-2)
  ⚠ This will trigger the pump relay if gas > 70%
- Flame sensor responds (briefly hold lighter at distance)
- Battery voltage reads approximately 7.4V–8.4V
```

---

## PART 5 — OTA UPDATES

### Step 14 — Push OTA Updates

```bash
# Build new firmware
idf.py build

# Host the binary on a local server
cd build
python3 -m http.server 8888

# In the dashboard → Modules tab → OTA Panel:
# Enter: http://YOUR_PC_IP:8888/VelocityOS.bin
# Click "Flash"
# Watch progress bar — robot reboots automatically
```

---

## TROUBLESHOOTING

| Problem | Solution |
|---|---|
| Won't connect to WiFi | Check SSID/password in menuconfig, 2.4GHz only |
| Motors not moving | Check motor driver wiring, verify STBY pin is HIGH |
| Ultrasonic reads 0 | Check 5V→3.3V voltage divider on ECHO pin |
| Dashboard won't connect | Ensure robot IP is correct, same WiFi network |
| Gas sensor always high | Allow 5-minute warm-up after power on |
| OTA fails | Ensure firmware binary URL is accessible from robot |
| ESTOP won't clear | Power cycle the robot |

---

## QUICK REFERENCE

```
Boot time:        ~4 seconds to WiFi connected
Telemetry rate:   20 Hz (50ms update interval)
Control latency:  <10ms via WebSocket
Scan duration:    30–90 seconds (area dependent)
API base URL:     http://ROBOT_IP/api/
WebSocket URL:    ws://ROBOT_IP/ws
Default password: Set in menuconfig (CONFIG_ADMIN_TOKEN)
E-Stop:           Always accessible, no auth required
```

---

*Powered by Aosmic Studio × Absolute Tech*
*Apache License 2.0*

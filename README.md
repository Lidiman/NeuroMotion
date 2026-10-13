# NeuroMotion 🧠⚡

> **Multimodal Early-Warning Microsleep & Drowsiness Detection System for Motorcyclists**

NeuroMotion is an edge-AI safety system engineered to detect drowsiness and microsleep episodes in real time. Designed specifically for motorcyclists, it combines on-device computer vision and inertial head-kinematics sensing across a distributed dual-microcontroller architecture using ultra-low latency wireless communication.

---

## 📌 System Architecture

NeuroMotion divides workloads across two specialized edge nodes communicating over **ESP-NOW**:

```
+------------------------------------+             ESP-NOW             +------------------------------------+
|       Vision Subsystem Node        |  (Eye State, Conf, Timestamp)   |       Decision Engine Node         |
|      (Seeed XIAO ESP32-S3 Sense)   | ----------------------------->  |       (ESP32-C3 Super Mini)        |
|  - OV2640 Camera (96x96 Grayscale) |                                 |  - GY-25 IMU (Yaw/Pitch/Roll UART) |
|  - TFLite Micro INT8 CNN Inference |                                 |  - Multimodal Evidence Fusion      |
|  - Real-time Eye State Extraction  |                                 |  - PWM Buzzer & Haptic Vibration   |
+------------------------------------+                                 +------------------------------------+
```

1. **Vision Subsystem (Sensor Node — Helmet/Visor Mounted):**
   - Captures frame buffers using the OV2640 camera.
   - Runs a quantized INT8 Convolutional Neural Network on **TensorFlow Lite Micro** to classify eye state (`OPEN` vs `CLOSED`).
   - Streams classification telemetry packets (`VisionPacket_t`) via ESP-NOW.

2. **Decision Engine (Alert Node — Handlebar/Body Mounted):**
   - Continuously receives telemetry packets via ESP-NOW.
   - Reads head orientation and kinematics (Pitch, Roll, Yaw) from a GY-25 IMU sensor over UART.
   - Evaluates multi-modal evidence (PERCLOS temporal window + sudden head slump dynamics).
   - Triggers tiered multi-stage warnings (PWM audio tone and haptic vibration motors).

---

## 🚀 Key Features

- **On-Device Edge AI:** Zero cloud dependency. Full inference latency ~30–45 ms per frame directly on the ESP32-S3.
- **Multimodal Evidence Fusion:** Eliminates false positives by correlating ocular closure duration with kinetic head-slump angles.
- **Low-Latency Ad-hoc Wireless:** Sub-5ms packet transmission via connectionless ESP-NOW protocol.
- **Tiered Warning System:** Graduated alert escalation from subtle haptic pulse to continuous 2 kHz high-frequency emergency alarms.

---

## 🛠 Hardware Specifications & Pinout

### 1. Vision Subsystem — Seeed Studio XIAO ESP32-S3 Sense
| Function / Peripheral | Pin / Configuration |
| :--- | :--- |
| **Camera Sensor** | OV2640 (DVP Interface) |
| **Camera XCLK / PCLK** | GPIO10 / GPIO13 |
| **Camera Data (Y2–Y9)** | GPIO15, 17, 18, 16, 14, 12, 11, 48 |
| **Camera Sync (VSYNC/HREF)** | GPIO38 / GPIO47 |
| **I2C SIOC / SIOD** | GPIO39 / GPIO40 |
| **Status Indicator LED** | GPIO21 |

### 2. Decision Engine — ESP32-C3 Super Mini
| Peripheral | Pin | Description |
| :--- | :--- | :--- |
| **Buzzer Output** | GPIO3 | 2 kHz PWM output (LEDC Channel 0) |
| **Vibration Motor** | GPIO4 | Haptic motor transistor gate |
| **GY-25 IMU RX** | GPIO20 | Connects to GY-25 TX (UART1 @ 115200 baud) |
| **GY-25 IMU TX** | GPIO21 | Connects to GY-25 RX |
| **Onboard Status LED** | GPIO8 | System status / Alert indicator |

---

## 📁 Repository Structure

```
NeuroMotion/
├── decision_engine/
│   └── decision_engine.ino        # Firmware for ESP32-C3 alert & fusion engine
├── vision_subsystem/
│   ├── vision_subsystem.ino       # Firmware for XIAO ESP32-S3 vision node
│   ├── camera_stream.h            # Camera streaming helper routines
│   └── model_data.h               # Quantized TFLite Micro C byte array model
├── shared/
│   └── config.h                   # Shared constants, thresholds, & MAC configs
└── README.md
```

---

## 🧠 Model Architecture & Artifacts

- **Model:** `NeuroMotionCNN.onnx` / `model_data.h` (TFLite Micro INT8)
- **Input Dimensions:** `96x96x1` (Grayscale ROI)
- **Output Classes:**
  - `0`: Eyes Open
  - `1`: Eyes Closed / Drowsy
- **Download Model Artifacts:** [NeuroMotionCNN.onnx (Google Drive)](https://drive.google.com/file/d/1KoTyLn6y7zxhs0Zp4TWKjgMLnNXi8L6G/view?usp=sharing)

---

## ⚙️ Configuration & Thresholds

All tunable parameters are centrally defined in `shared/config.h`:

```c
#define RISK_THRESHOLD        0.85f   // Aggregate risk threshold to trigger alert
#define MIN_CLOSURE_MS        1200    // Minimum closure duration before scoring (ms)
#define MAX_CLOSURE_MS        2000    // Full temporal score duration (ms)

// Fusion Weights (sum = 1.0)
#define W_VISUAL              0.5f    // Real-time eye state weight
#define W_TEMPORAL            0.3f    // Sustained closure (PERCLOS) weight
#define W_ORIENT              0.2f    // Head tilt / slump kinematics weight
```

---

## 🔧 Getting Started & Flashing

### Prerequisites
- [Arduino IDE](https://www.arduino.cc/en/software) or [arduino-cli](https://arduino.github.io/arduino-cli/)
- ESP32 Board Package (`esp32` by Espressif Systems >= 2.0.14)
- **Libraries Required:**
  - `Chirale_TensorFlowLite` (or standard `TensorFlowLite_ESP32`)
  - `esp_camera`

### Build & Flash

1. **Configure MAC Addresses:**
   Obtain the Wi-Fi STA MAC address of each board and update `shared/config.h`:
   ```c
   #define DECISION_ENGINE_MAC  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}
   #define VISION_SUBSYSTEM_MAC {0x11, 0x22, 0x33, 0x44, 0x55, 0x66}
   ```

2. **Flash Vision Subsystem (XIAO ESP32-S3):**
   ```bash
   arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc,PSRAM=qio vision_subsystem/
   arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc,PSRAM=qio vision_subsystem/
   ```

3. **Flash Decision Engine (ESP32-C3):**
   ```bash
   arduino-cli compile --fqbn esp32:esp32:esp32c3 decision_engine/
   arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32c3 decision_engine/
   ```

---

## 📄 License

This project is licensed under the MIT License.

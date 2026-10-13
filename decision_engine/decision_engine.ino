/**
 * NeuroMotion – Decision Engine (Helmet Unit)
 * Board  : ESP32-C3 Super Mini
 * Sensor : GY-25 IMU (Attitude Estimation via UART)
 * Actuator: Piezo Buzzer (GPIO 3) + Haptic Vibration Motor (GPIO 4)
 *
 * Capabilities:
 *   1. Seamless ESP-NOW packet reception from Vision Subsystem.
 *   2. Auto-calibration of baseline riding posture (Pitch_0, Roll_0).
 *   3. 8-frame Vision & 10-sample Motion Sliding Window Fusion.
 *   4. False-Positive Filtering:
 *      - Glances/Speed Bumps: Brief acceleration spikes ignored without eye closure.
 *      - Cornering/Leaning: Coordinated Roll without Pitch nod ignored.
 *      - Natural Blinks: Durations < 400 ms disregarded.
 *   5. Final Decision Output:
 *      - <= 50% Confidence: NORMAL (All actuators OFF).
 *      - > 50% Confidence: DROWSY / MICROSLEEP (Pulsed Buzzer + Haptic ON).
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "../shared/config.h"

// ─── Pin Definition (ESP32-C3 Super Mini) ─────────────────────
#define BUZZER_PIN   5  // Buzzer pin 5
#define VIBE_PIN     4  // Vibrator pin 4
#define GY25_RX_PIN  7  // ESP32-C3 RX (Connect to GY-25 TX pin 7)
#define GY25_TX_PIN  6  // ESP32-C3 TX (Connect to GY-25 RX pin 6)
#define LED_PIN      8  // Onboard Blue LED

// ─── LEDC PWM Configuration ──────────────────────────────────
#define BUZZER_CHANNEL  0
#define BUZZER_RES_BITS 8

// ─── Sliding Window Ring Buffers ──────────────────────────────
static volatile VisionPacket_t latestPkt;
static volatile bool newVisionPkt = false;

// Vision History Ring Buffer
static uint8_t  visionWindow[VISION_WINDOW_SIZE] = {0};
static uint8_t  visionIdx = 0;
static uint32_t lastVisionPktTime = 0;

// Motion (GY-25) History Ring Buffer
typedef struct {
    float pitch;
    float roll;
    float yaw;
    uint32_t timestamp;
} IMUSample_t;

static IMUSample_t motionWindow[MOTION_WINDOW_SIZE] = {};
static uint8_t     motionIdx = 0;

// Current & Baseline Orientations
static float curYaw = 0.0f, curPitch = 0.0f, curRoll = 0.0f;
static float basePitch = 0.0f, baseRoll = 0.0f;
static bool  baselineCalibrated = false;
static float calibPitchSum = 0.0f;
static float calibRollSum  = 0.0f;
static uint16_t calibCount = 0;
#define CALIB_TARGET_SAMPLES 20

// Temporal Eye Closure Tracking
static uint32_t closureStartMs = 0;
static uint32_t closureDuration = 0;
static bool     eyeCurrentlyClosed = false;

// Actuator & Alert States
static bool alertActive = false;
static uint32_t lastAlertToggle = 0;
static bool alertStateToggle = false;
static bool visionConnected = false;

HardwareSerial gy25Serial(1);

// ─── ESP-NOW Receive Callback ─────────────────────────────────
void ARDUINO_ISR_ATTR onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (len == sizeof(VisionPacket_t)) {
        memcpy((void*)&latestPkt, data, sizeof(VisionPacket_t));
        newVisionPkt = true;
        visionConnected = true;
    }
}

void initEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Init failed.");
        return;
    }
    esp_now_register_recv_cb(onDataRecv);
    Serial.printf("[ESP-NOW] Ready | MAC: %s\n", WiFi.macAddress().c_str());
}

// ─── GY-25 Stream Parser (Hardware UART1 on GPIO 7 RX / GPIO 6 TX) ─
static uint32_t gy25RawByteCount = 0;

void initGY25() {
    // Standard GY-25 hardware UART on GPIO 7 (RX) and GPIO 6 (TX)
    gy25Serial.begin(9600, SERIAL_8N1, GY25_RX_PIN, GY25_TX_PIN);
    delay(100);

    const uint8_t initCmd[] = {0xA5, 0x52, 0xA5, 0x53};
    gy25Serial.write(initCmd, sizeof(initCmd));
    delay(50);
}

void readGY25() {
    static uint32_t lastPing = 0;
    if (!baselineCalibrated && (millis() - lastPing > 1500)) {
        lastPing = millis();
        const uint8_t pingCmd[] = {0xA5, 0x52, 0xA5, 0x53};
        gy25Serial.write(pingCmd, sizeof(pingCmd));
    }

    static uint8_t buf[8];
    static uint8_t idx = 0;

    while (gy25Serial.available()) {
        gy25RawByteCount++;
        uint8_t b = gy25Serial.read();

        if (idx == 0) {
            if (b == 0xAA) {
                buf[0] = b;
                idx = 1;
            }
        } else if (idx == 1) {
            if (b == 0x55 || b == 0xAA) {
                buf[1] = b;
                idx = 2;
            } else if (b == 0xAA) {
                idx = 1;
            } else {
                idx = 0;
            }
        } else {
            buf[idx++] = b;
            if (idx >= 8) {
                int16_t yRaw = (int16_t)((buf[2] << 8) | buf[3]);
                int16_t pRaw = (int16_t)((buf[4] << 8) | buf[5]);
                int16_t rRaw = (int16_t)((buf[6] << 8) | buf[7]);

                float p = (float)pRaw / 100.0f;
                float r = (float)rRaw / 100.0f;
                float y = (float)yRaw / 100.0f;

                if (fabs(p) <= 180.0f && fabs(r) <= 180.0f) {
                    curYaw   = y;
                    curPitch = p;
                    curRoll  = r;

                    if (!baselineCalibrated) {
                        calibPitchSum += curPitch;
                        calibRollSum  += curRoll;
                        calibCount++;
                        if (calibCount >= CALIB_TARGET_SAMPLES) {
                            basePitch = calibPitchSum / (float)CALIB_TARGET_SAMPLES;
                            baseRoll  = calibRollSum  / (float)CALIB_TARGET_SAMPLES;
                            baselineCalibrated = true;
                            Serial.printf("\n[CALIB] Baseline Locked -> Pitch_0: %.2f deg | Roll_0: %.2f deg (Bytes Recv: %lu)\n",
                                basePitch, baseRoll, gy25RawByteCount);
                        }
                    } else {
                        motionWindow[motionIdx % MOTION_WINDOW_SIZE] = {curPitch, curRoll, curYaw, millis()};
                        motionIdx++;
                    }
                }
                idx = 0;
            }
        }
    }
}

// ─── Variance Computation Helper ─────────────────────────────
float computePitchVariance() {
    float mean = 0.0f;
    for (int i = 0; i < MOTION_WINDOW_SIZE; i++) mean += motionWindow[i].pitch;
    mean /= (float)MOTION_WINDOW_SIZE;
    float var = 0.0f;
    for (int i = 0; i < MOTION_WINDOW_SIZE; i++) {
        float diff = motionWindow[i].pitch - mean;
        var += diff * diff;
    }
    return var / (float)MOTION_WINDOW_SIZE;
}

// ─── Multi-Modal Evidence Fusion & False Positive Filter ─────
float evaluateMultiModalDecision(uint8_t currentEyeState, uint8_t closedPct) {
    // 1. Push Vision Score into sliding window
    visionWindow[visionIdx % VISION_WINDOW_SIZE] = closedPct;
    visionIdx++;

    // Calculate averaged visual closure probability over window
    float avgVisualClosure = 0.0f;
    for (int i = 0; i < VISION_WINDOW_SIZE; i++) {
        avgVisualClosure += (float)visionWindow[i];
    }
    avgVisualClosure = (avgVisualClosure / (float)VISION_WINDOW_SIZE) / 100.0f; // [0.0 - 1.0]

    // 2. Temporal Score Calculation
    float temporalScore = 0.0f;
    if (closureDuration >= MIN_CLOSURE_MS) {
        temporalScore = (float)(closureDuration - MIN_CLOSURE_MS) / (float)(MAX_CLOSURE_MS - MIN_CLOSURE_MS);
        temporalScore = constrain(temporalScore, 0.0f, 1.0f);
    }

    // 3. Motion Posture & Stupor Evaluation
    float deltaPitch = curPitch - basePitch;
    float deltaRoll  = curRoll - baseRoll;
    float pitchVar   = computePitchVariance();

    float motionScore = 0.0f;

    // Head Nodding Detection (Pitch down)
    if (deltaPitch <= THRESH_PITCH_NOD) {
        motionScore += 0.70f;
    }
    // Head Tilting / Slumping Detection (Roll side)
    if (fabs(deltaRoll) >= THRESH_ROLL_TILT) {
        motionScore += 0.50f;
    }
    // Stupor / Frozen Head state during eye closure
    if (pitchVar <= THRESH_STUPOR_VAR && avgVisualClosure >= 0.40f) {
        motionScore += 0.40f;
    }
    motionScore = constrain(motionScore, 0.0f, 1.0f);

    // ─── False-Positive Suppression Rules ────────────────────
    // Rule 1: Natural Blink Filter (Duration < 400 ms is disregarded)
    if (closureDuration < 400 && currentEyeState == EYE_CLOSED) {
        avgVisualClosure *= 0.50f;
    }

    // Rule 2: Cornering Leaning without Eye Closure (Rider is actively looking through turn)
    if (fabs(deltaRoll) >= 15.0f && avgVisualClosure < 0.30f) {
        motionScore = 0.0f; // Motorcycle lean is not drowsiness
    }

    // Rule 3: Speed Bumps / Road Vibration (High variance without head tilt)
    if (pitchVar > 8.0f && avgVisualClosure < 0.35f) {
        motionScore = 0.0f;
    }

    // 4. Multi-Modal Fused Confidence Calculation
    float finalConfidence = (W_VISUAL * avgVisualClosure) +
                            (W_TEMPORAL * temporalScore) +
                            (W_MOTION * motionScore);

    return constrain(finalConfidence, 0.0f, 1.0f);
}

// ─── Active Low/High Actuator Logic ───────────────────────────
// All modules (Buzzer, Vibration driver, and Blue LED) on ESP32-C3 are ACTIVE-LOW (LOW=ON, HIGH=OFF)
#define VIBE_ON       LOW
#define VIBE_OFF      HIGH

#define LED_ON        LOW
#define LED_OFF       HIGH

#define BUZZER_ON     LOW
#define BUZZER_OFF    HIGH

// ─── Actuator Driver (Buzzer + Haptic Vibration Motor) ────────
void triggerActuators(bool alertOn) {
    alertActive = alertOn;
    if (!alertActive) {
        digitalWrite(BUZZER_PIN, BUZZER_OFF);
        digitalWrite(VIBE_PIN, VIBE_OFF);
        digitalWrite(LED_PIN, LED_OFF);
        alertStateToggle = false;
    }
}

void updatePulsedAlert() {
    if (!alertActive) {
        digitalWrite(BUZZER_PIN, BUZZER_OFF);
        digitalWrite(VIBE_PIN, VIBE_OFF);
        digitalWrite(LED_PIN, LED_OFF);
        return;
    }

    uint32_t now = millis();
    uint32_t halfPeriodMs = 1000 / (ALERT_PULSE_HZ * 2);

    if (now - lastAlertToggle >= halfPeriodMs) {
        alertStateToggle = !alertStateToggle;
        if (alertStateToggle) {
            digitalWrite(BUZZER_PIN, BUZZER_ON);
            digitalWrite(VIBE_PIN, VIBE_ON);
            digitalWrite(LED_PIN, LED_ON);
        } else {
            digitalWrite(BUZZER_PIN, BUZZER_OFF);
            digitalWrite(VIBE_PIN, VIBE_OFF);
            digitalWrite(LED_PIN, LED_OFF);
        }
        lastAlertToggle = now;
    }
}

// ─── Setup & Loop ─────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("==================================================");
    Serial.println("  NeuroMotion - Decision Engine (ESP32-C3)        ");
    Serial.println("==================================================");

    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(VIBE_PIN,   OUTPUT);
    pinMode(LED_PIN,    OUTPUT);

    // Initial State: Force ALL Actuators & LED OFF
    digitalWrite(BUZZER_PIN, BUZZER_OFF);
    digitalWrite(VIBE_PIN, VIBE_OFF);
    digitalWrite(LED_PIN, LED_OFF);

    // Initialize GY-25 UART on Pin 20 (RX) & 21 (TX)
    initGY25();
    Serial.println("[GY-25] UART initialized on GPIO 20/21 (auto output mode sent)");

    initEspNow();

    Serial.println("[SYSTEM] Decision Engine running. Calibrating posture...");
}

void loop() {
    readGY25();

    // If vision subsystem is not connected yet or timed out (> 2s), FORCE ALL ACTUATORS OFF
    if (!visionConnected || (millis() - lastVisionPktTime > 2000)) {
        triggerActuators(false);
        digitalWrite(BUZZER_PIN, BUZZER_OFF);
        digitalWrite(VIBE_PIN, VIBE_OFF);
        digitalWrite(LED_PIN, LED_OFF);
    }

    if (newVisionPkt) {
        newVisionPkt = false;
        lastVisionPktTime = millis();

        uint8_t state = latestPkt.eyeState;
        uint8_t conf  = latestPkt.closedProbPct;

        // Update Eye Closure Duration Counter
        if (state == EYE_CLOSED) {
            if (!eyeCurrentlyClosed) {
                closureStartMs     = millis();
                eyeCurrentlyClosed = true;
            }
            closureDuration = millis() - closureStartMs;
        } else {
            eyeCurrentlyClosed = false;
            closureDuration    = 0;
            closureStartMs     = 0;
        }

        // Multi-Modal Fusion
        float fusedConfidence = evaluateMultiModalDecision(state, conf);
        uint8_t confidencePct = (uint8_t)(fusedConfidence * 100.0f);

        // Final Binary Decision: <= 50% = NORMAL, > 50% = DROWSY / MICROSLEEP
        bool isDrowsy = (fusedConfidence > FINAL_RISK_THRESHOLD);
        triggerActuators(isDrowsy);

#if DEBUG_SERIAL
        float display_dPitch = baselineCalibrated ? (curPitch - basePitch) : 0.0f;
        float display_dRoll  = baselineCalibrated ? (curRoll - baseRoll)   : 0.0f;

        Serial.printf("[DECISION] VisConf: %3d%% | Dur: %4lu ms | dPitch: %5.1f | dRoll: %5.1f | FUSED: %3d%% | STATUS: %s\n",
            conf, closureDuration,
            display_dPitch, display_dRoll,
            confidencePct,
            isDrowsy ? ">>> ALARM (MICROSLEEP) <<<" : "NORMAL");
#endif
    }

    updatePulsedAlert();
}

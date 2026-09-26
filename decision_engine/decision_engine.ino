/**
 * NeuroMotion – Decision Engine
 * Board: ESP32-C3 Super Mini
 *
 * Function:
 *   - Receives VisionPacket_t from Vision Subsystem via ESP-NOW.
 *   - Reads GY-25 IMU (yaw/pitch/roll) via UART.
 *   - Computes microsleep risk score using Multi-Modal Evidence Fusion.
 *   - Triggers buzzer (2 kHz PWM) + vibration motor if risk > threshold.
 *
 * Pins (adjust to your wiring):
 *   BUZZER_PIN    : GPIO3  (PWM capable)
 *   VIBE_PIN      : GPIO4  (GPIO for vibration motor transistor base)
 *   GY25_RX_PIN   : GPIO20 (UART1 RX, connect to GY-25 TX)
 *   GY25_TX_PIN   : GPIO21 (UART1 TX, connect to GY-25 RX — usually not needed)
 *   LED_PIN       : GPIO8  (built-in LED on most C3 Super Mini)
 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "../shared/config.h"

// ─── Pin Assignments ──────────────────────────────────────────────────────────
#define BUZZER_PIN   3
#define VIBE_PIN     4
#define GY25_RX_PIN  20
#define GY25_TX_PIN  21
#define LED_PIN      8

// ─── LEDC (PWM) for Buzzer ────────────────────────────────────────────────────
#define BUZZER_CHANNEL  0
#define BUZZER_RES_BITS 8

// ─── ESP-NOW Packets ──────────────────────────────────────────────────────────
typedef struct {
    uint8_t  eyeState;
    uint8_t  confidence;
    uint32_t timestamp;
} VisionPacket_t;

// ─── State ────────────────────────────────────────────────────────────────────
static volatile bool     newPacketReady = false;
static volatile VisionPacket_t latestPkt;

// Eye closure tracking
static uint32_t closureStartMs  = 0;
static uint32_t closureDuration = 0;
static bool     eyeCurrentlyClosed = false;

// Head orientation (GY-25)
static float currentYaw   = 0.0f;
static float currentPitch = 0.0f;
static float currentRoll  = 0.0f;

// Orientation variance window (1 s @ ~20 Hz = 20 samples)
#define ORIENT_WINDOW 20
static float pitchHistory[ORIENT_WINDOW] = {};
static float rollHistory[ORIENT_WINDOW]  = {};
static int   orientIdx = 0;

// Alert state
static bool alertActive = false;
static unsigned long lastAlertToggle = 0;
static bool alertBuzzerOn = false;

// ─── ESP-NOW Receive Callback ─────────────────────────────────────────────────
void ARDUINO_ISR_ATTR onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (len == sizeof(VisionPacket_t)) {
        memcpy((void*)&latestPkt, data, sizeof(VisionPacket_t));
        newPacketReady = true;
    }
}

// ─── GY-25 via UART ───────────────────────────────────────────────────────────
// GY-25 outputs: "Yaw:XXX.XX,Pitch:XXX.XX,Roll:XXX.XX\r\n" at ~20 Hz
// Some modules use a simpler binary protocol — adjust parseGY25Line() as needed.
HardwareSerial gy25Serial(1);

void parseGY25Line(const String& line) {
    // Expected format: "Yaw:12.34,Pitch:-5.67,Roll:0.12"
    int yIdx = line.indexOf("Yaw:");
    int pIdx = line.indexOf("Pitch:");
    int rIdx = line.indexOf("Roll:");
    if (yIdx < 0 || pIdx < 0 || rIdx < 0) return;

    currentYaw   = line.substring(yIdx + 4, pIdx - 1).toFloat();
    currentPitch = line.substring(pIdx + 6, rIdx - 1).toFloat();
    currentRoll  = line.substring(rIdx + 5).toFloat();
}

void readIMU() {
    while (gy25Serial.available()) {
        String line = gy25Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 5) {
            parseGY25Line(line);
            // Store in variance window
            pitchHistory[orientIdx % ORIENT_WINDOW] = currentPitch;
            rollHistory [orientIdx % ORIENT_WINDOW] = currentRoll;
            orientIdx++;
        }
    }
}

// ─── Variance Computation ─────────────────────────────────────────────────────
float computeVariance(const float* arr, int n) {
    float mean = 0.0f;
    for (int i = 0; i < n; i++) mean += arr[i];
    mean /= n;
    float var = 0.0f;
    for (int i = 0; i < n; i++) var += (arr[i] - mean) * (arr[i] - mean);
    return var / n;
}

// ─── Risk Score Computation ───────────────────────────────────────────────────
float computeRiskScore(uint8_t conf, uint32_t closureDur) {
    // Visual score: confidence that eye is closed (0.0–1.0)
    float visScore  = conf / 100.0f;

    // Temporal score: how long eye has been closed, normalized
    float tempScore = (float)closureDur / (float)MAX_CLOSURE_MS;
    if (tempScore > 1.0f) tempScore = 1.0f;

    // Orientation score: low variance = head is still = riskier
    float pitchVar  = computeVariance(pitchHistory, ORIENT_WINDOW);
    float rollVar   = computeVariance(rollHistory,  ORIENT_WINDOW);
    float totalVar  = pitchVar + rollVar;
    // Normalize: variance > 5 degrees^2 → head is moving → low orient risk
    float normVar   = totalVar / 5.0f;
    if (normVar > 1.0f) normVar = 1.0f;
    float orientScore = 1.0f - normVar;  // high score = head still = risky

    float risk = W_VISUAL * visScore + W_TEMPORAL * tempScore + W_ORIENT * orientScore;

#if DEBUG_SERIAL
    Serial.printf("[RISK] vis=%.2f temp=%.2f orient=%.2f → score=%.3f\n",
        visScore, tempScore, orientScore, risk);
#endif
    return risk;
}

// ─── Alert Control ───────────────────────────────────────────────────────────
void setAlert(bool active) {
    if (alertActive == active) return;
    alertActive = active;
    if (!active) {
        ledcWrite(BUZZER_CHANNEL, 0);   // buzzer off
        digitalWrite(VIBE_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
        alertBuzzerOn = false;
    }
}

void updateAlertPulse() {
    if (!alertActive) return;
    unsigned long now = millis();
    unsigned long halfPeriodMs = 1000 / (ALERT_PULSE_HZ * 2);  // 250 ms for 2 Hz
    if (now - lastAlertToggle >= halfPeriodMs) {
        alertBuzzerOn = !alertBuzzerOn;
        if (alertBuzzerOn) {
            // Duty cycle: ALERT_DUTY_PCT % of 255
            ledcWrite(BUZZER_CHANNEL, (uint32_t)(255 * ALERT_DUTY_PCT / 100));
            digitalWrite(VIBE_PIN, HIGH);
            digitalWrite(LED_PIN, HIGH);
        } else {
            ledcWrite(BUZZER_CHANNEL, 0);
            digitalWrite(VIBE_PIN, LOW);
            digitalWrite(LED_PIN, LOW);
        }
        lastAlertToggle = now;
    }
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[NeuroMotion] Decision Engine booting...");

    // GPIO
    pinMode(VIBE_PIN, OUTPUT);
    pinMode(LED_PIN,  OUTPUT);
    digitalWrite(VIBE_PIN, LOW);
    digitalWrite(LED_PIN,  LOW);

    // Buzzer via LEDC
    ledcSetup(BUZZER_CHANNEL, ALERT_TONE_HZ, BUZZER_RES_BITS);
    ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
    ledcWrite(BUZZER_CHANNEL, 0);

    // GY-25 UART (9600 baud, 8N1)
    gy25Serial.begin(9600, SERIAL_8N1, GY25_RX_PIN, GY25_TX_PIN);
    Serial.println("[GY25] UART started at 9600");

    // Wi-Fi STA for ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    Serial.print("[WIFI] MAC: ");
    Serial.println(WiFi.macAddress());

    // ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[FATAL] ESP-NOW init failed.");
        while (1) delay(1000);
    }
    esp_now_register_recv_cb(onDataRecv);

    // Register Vision Subsystem as peer (optional — only needed if sending back ControlPackets)
    uint8_t visionMac[] = VISION_SUBSYSTEM_MAC;
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, visionMac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    Serial.println("[NeuroMotion] Decision Engine ready. Waiting for packets...");
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
    // Read IMU continuously
    readIMU();

    // Process new vision packet
    if (newPacketReady) {
        newPacketReady = false;

        uint8_t  state = latestPkt.eyeState;
        uint8_t  conf  = latestPkt.confidence;
        uint32_t ts    = latestPkt.timestamp;

        // Update closure timer
        if (state == EYE_CLOSED) {
            if (!eyeCurrentlyClosed) {
                closureStartMs    = millis();
                eyeCurrentlyClosed = true;
            }
            closureDuration = millis() - closureStartMs;
        } else {
            eyeCurrentlyClosed = false;
            closureDuration    = 0;
            closureStartMs     = 0;
        }

        // Compute risk
        float risk = computeRiskScore(conf, closureDuration);

        // Trigger / clear alert
        bool triggerAlert = (risk > RISK_THRESHOLD) && (closureDuration > MIN_CLOSURE_MS);
        setAlert(triggerAlert);

#if DEBUG_SERIAL
        Serial.printf("[PKT] state=%s conf=%d%% dur=%ums risk=%.3f alert=%s\n",
            state == EYE_CLOSED ? "CLOSED" : "OPEN",
            conf, closureDuration, risk,
            triggerAlert ? "ON" : "off");
#endif
    }

    // Pulse the alert if active
    updateAlertPulse();
}

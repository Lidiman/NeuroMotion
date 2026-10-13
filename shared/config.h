/**
 * NeuroMotion - Shared Configuration
 * Compile-time constants for Vision Subsystem (ESP32-S3) and Decision Engine (ESP32-C3).
 */

#ifndef NEUROMOTION_CONFIG_H
#define NEUROMOTION_CONFIG_H

#include <stdint.h>

// ─── ESP-NOW Peer MAC Addresses ──────────────────────────────────────────────
// Broadcast address (0xFF, ...) allows out-of-the-box pairing without hardcoding
#define DECISION_ENGINE_MAC  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
#define VISION_SUBSYSTEM_MAC {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

// ─── Shared Telemetry Packet ──────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    uint8_t  eyeState;       // 0: EYE_OPEN, 1: EYE_CLOSED
    uint8_t  closedProbPct;  // 0 - 100% confidence of eye closure
    uint16_t latencyMs;      // Camera & inference latency in ms
    uint32_t timestamp;      // Millis timestamp from vision subsystem
} VisionPacket_t;

// ─── Model Input Dimensions ───────────────────────────────────────────────────
#define MODEL_INPUT_WIDTH    160
#define MODEL_INPUT_HEIGHT   120
#define MODEL_INPUT_CHANNELS 1   // grayscale

// ─── Eye State Labels ────────────────────────────────────────────────────────
#define EYE_OPEN   0
#define EYE_CLOSED 1

// ─── Sliding Window Evaluation Sizes ─────────────────────────────────────────
#define VISION_WINDOW_SIZE   5   // Evaluate last 5 frames (~150-200 ms history)
#define MOTION_WINDOW_SIZE   5   // Evaluate last 5 IMU samples (~250 ms history)

// ─── Decision Engine Multi-Modal Fusion Weights ──────────────────────────────
#define W_VISUAL             0.45f  // Weight for vision closure probability
#define W_TEMPORAL           0.35f  // Weight for closure duration (PERCLOS-like)
#define W_MOTION             0.20f  // Weight for head nodding/tilting posture

// ─── Decision Engine Scoring Thresholds ──────────────────────────────────────
#define FINAL_RISK_THRESHOLD 0.50f  // > 50% = Drowsy/Microsleep Alert, <= 50% = Normal
#define MIN_CLOSURE_MS       800    // Minimum duration before temporal risk starts accumulating
#define MAX_CLOSURE_MS       1800   // Closure duration reaching 100% temporal risk

// ─── GY-25 IMU Orientation Thresholds (Degrees) ──────────────────────────────
#define THRESH_PITCH_NOD     -20.0f // Head nod down beyond baseline
#define THRESH_ROLL_TILT     25.0f  // Head side tilt beyond baseline
#define THRESH_STUPOR_VAR    1.5f   // Head rigidly frozen with closed eyes (microsleep stupor)
#define THRESH_JERK_DELTA    22.0f  // Sudden snapback recovery movement

// ─── Actuator Parameters ─────────────────────────────────────────────────────
#define ALERT_TONE_HZ        2000
#define ALERT_PULSE_HZ       3      // Pulse rate in Hz (fast alert)
#define ALERT_DUTY_PCT       50

// ─── System Debug ────────────────────────────────────────────────────────────
#define DEBUG_SERIAL         1

#endif // NEUROMOTION_CONFIG_H

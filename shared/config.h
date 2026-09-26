/**
 * NeuroMotion - Shared Configuration
 * Compile-time constants for both Vision Subsystem and Decision Engine.
 */

#ifndef NEUROMOTION_CONFIG_H
#define NEUROMOTION_CONFIG_H

// ─── ESP-NOW MAC Addresses ────────────────────────────────────────────────────
// Set these to the actual MAC addresses of your boards.
// Vision → broadcasts to Decision Engine MAC
#define DECISION_ENGINE_MAC  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}  // replace with C3 MAC
#define VISION_SUBSYSTEM_MAC {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}  // replace with S3 MAC

// ─── Model Input ─────────────────────────────────────────────────────────────
#define MODEL_INPUT_WIDTH   96
#define MODEL_INPUT_HEIGHT  96
#define MODEL_INPUT_CHANNELS 1   // grayscale

// ─── Eye State Labels ────────────────────────────────────────────────────────
#define EYE_OPEN   0
#define EYE_CLOSED 1

// ─── Decision Engine Thresholds ──────────────────────────────────────────────
#define RISK_THRESHOLD        0.85f
#define MIN_CLOSURE_MS        1200     // minimum closure duration to start scoring
#define MAX_CLOSURE_MS        2000     // closure duration = 100% temporal score

// ─── Fusion Weights (must sum to 1.0) ────────────────────────────────────────
#define W_VISUAL    0.5f
#define W_TEMPORAL  0.3f
#define W_ORIENT    0.2f

// ─── Alert Parameters ────────────────────────────────────────────────────────
#define ALERT_TONE_HZ     2000
#define ALERT_PULSE_HZ    2       // on/off blink frequency
#define ALERT_DUTY_PCT    50

// ─── Vision Loop Rate ─────────────────────────────────────────────────────────
#define VISION_LOOP_MS    33      // ~30 fps

// ─── Serial Debug ─────────────────────────────────────────────────────────────
#define DEBUG_SERIAL      1       // set 0 to disable Serial.print

#endif // NEUROMOTION_CONFIG_H

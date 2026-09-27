/*
 * ============================================================
 * NeuroMotion – Vision Subsystem (Eye State Tester)
 * Board  : Seeed Studio XIAO ESP32-S3 Sense
 * Camera : OV2640
 *
 * Mode   : Serial Monitor Tester
 *   - Runs TFLite Micro INT8 model on 96x96 grayscale frames
 *   - Prints real-time eye state, confidence, duration, latency
 *   - Onboard LED ON = CLOSED (drowsy)
 *   - Broadcasts VisionPacket_t via ESP-NOW to Decision Engine
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "esp_camera.h"

#include <Chirale_TensorFlowLite.h>
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "../shared/config.h"
#include "model_data.h"


// ============================================================
// CAMERA PINS (XIAO ESP32-S3 Sense)
// ============================================================

#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   10
#define SIOD_GPIO_NUM   40
#define SIOC_GPIO_NUM   39
#define Y9_GPIO_NUM     48
#define Y8_GPIO_NUM     11
#define Y7_GPIO_NUM     12
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     16
#define Y4_GPIO_NUM     18
#define Y3_GPIO_NUM     17
#define Y2_GPIO_NUM     15
#define VSYNC_GPIO_NUM  38
#define HREF_GPIO_NUM   47
#define PCLK_GPIO_NUM   13

#define LED_PIN         21


// ============================================================
// TYPES
// ============================================================

typedef struct {
    uint8_t  eyeState;
    uint8_t  confidence;
    uint32_t timestamp;
} VisionPacket_t;

typedef struct {
    uint8_t  state;
    uint8_t  confidence;
    uint32_t latencyMs;
} InferenceResult;


// ============================================================
// TFLite Micro
// ============================================================

namespace {
    const tflite::Model*      tfl_model       = nullptr;
    tflite::MicroInterpreter* tfl_interpreter  = nullptr;
    TfLiteTensor*             tfl_input        = nullptr;

    constexpr int kArenaSize = 96 * 1024;
    uint8_t* tensor_arena = nullptr;

    tflite::MicroMutableOpResolver<8> resolver;
}


// ============================================================
// ESP-NOW
// ============================================================

static bool espnowReady = false;

void onDataSent(const esp_now_send_info_t* info, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("[ESP-NOW] Send FAILED");
    }
}

void initEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Init failed — continuing without ESP-NOW");
        return;
    }

    esp_now_register_send_cb(onDataSent);

    uint8_t peerMac[] = DECISION_ENGINE_MAC;
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, peerMac, 6);
    peer.channel = 0;
    peer.encrypt = false;

    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("[ESP-NOW] Add peer failed");
        return;
    }

    espnowReady = true;
    Serial.print("[ESP-NOW] Ready | MAC: ");
    Serial.println(WiFi.macAddress());
}

void sendVisionPacket(uint8_t state, uint8_t conf) {
    if (!espnowReady) return;
    VisionPacket_t pkt;
    pkt.eyeState   = state;
    pkt.confidence = conf;
    pkt.timestamp  = millis();
    uint8_t peerMac[] = DECISION_ENGINE_MAC;
    esp_now_send(peerMac, (uint8_t*)&pkt, sizeof(pkt));
}


// ============================================================
// CAMERA INIT
// ============================================================

bool initCamera() {
    camera_config_t cfg = {};
    cfg.ledc_channel  = LEDC_CHANNEL_0;
    cfg.ledc_timer    = LEDC_TIMER_0;
    cfg.pin_d0        = Y2_GPIO_NUM;
    cfg.pin_d1        = Y3_GPIO_NUM;
    cfg.pin_d2        = Y4_GPIO_NUM;
    cfg.pin_d3        = Y5_GPIO_NUM;
    cfg.pin_d4        = Y6_GPIO_NUM;
    cfg.pin_d5        = Y7_GPIO_NUM;
    cfg.pin_d6        = Y8_GPIO_NUM;
    cfg.pin_d7        = Y9_GPIO_NUM;
    cfg.pin_xclk      = XCLK_GPIO_NUM;
    cfg.pin_pclk      = PCLK_GPIO_NUM;
    cfg.pin_vsync     = VSYNC_GPIO_NUM;
    cfg.pin_href      = HREF_GPIO_NUM;
    cfg.pin_sccb_sda  = SIOD_GPIO_NUM;
    cfg.pin_sccb_scl  = SIOC_GPIO_NUM;
    cfg.pin_pwdn      = PWDN_GPIO_NUM;
    cfg.pin_reset     = RESET_GPIO_NUM;
    cfg.xclk_freq_hz  = 20000000;
    cfg.pixel_format  = PIXFORMAT_GRAYSCALE;
    cfg.frame_size    = FRAMESIZE_96X96;
    cfg.jpeg_quality  = 12;
    cfg.fb_count      = 1;
    cfg.grab_mode     = CAMERA_GRAB_LATEST;
    cfg.fb_location   = CAMERA_FB_IN_PSRAM;

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        Serial.printf("[CAM] Init failed: 0x%x\n", err);
        return false;
    }
    Serial.println("[CAM] OK — GRAYSCALE 96x96");
    return true;
}


// ============================================================
// MODEL INIT
// ============================================================

bool initModel() {
    tensor_arena = (uint8_t*)heap_caps_malloc(kArenaSize, MALLOC_CAP_SPIRAM);
    if (!tensor_arena) {
        Serial.println("[MODEL] PSRAM alloc failed");
        return false;
    }

    tfl_model = tflite::GetModel(g_model);
    if (tfl_model->version() != TFLITE_SCHEMA_VERSION) {
        Serial.printf("[MODEL] Schema mismatch: got %d expected %d\n",
            tfl_model->version(), TFLITE_SCHEMA_VERSION);
        return false;
    }

    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddMaxPool2D();
    resolver.AddReshape();
    resolver.AddFullyConnected();
    resolver.AddSoftmax();
    resolver.AddQuantize();
    resolver.AddDequantize();

    static tflite::MicroInterpreter static_interp(
        tfl_model, resolver, tensor_arena, kArenaSize);
    tfl_interpreter = &static_interp;

    if (tfl_interpreter->AllocateTensors() != kTfLiteOk) {
        Serial.println("[MODEL] AllocateTensors failed");
        return false;
    }

    tfl_input = tfl_interpreter->input(0);

    Serial.printf("[MODEL] OK — arena used: %u / %d bytes\n",
        tfl_interpreter->arena_used_bytes(), kArenaSize);
    return true;
}


// ============================================================
// PREPROCESSING
// ============================================================

void preprocessFrame(const uint8_t* src) {
    int8_t* dst = tfl_input->data.int8;
    int     n   = MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT;
    for (int i = 0; i < n; i++) {
        dst[i] = (int8_t)((int)src[i] - 128);
    }
}


// ============================================================
// INFERENCE
// ============================================================

InferenceResult runInference(const camera_fb_t* fb) {
    preprocessFrame(fb->buf);

    uint32_t t0 = millis();
    tfl_interpreter->Invoke();
    uint32_t latency = millis() - t0;

    TfLiteTensor* out = tfl_interpreter->output(0);

    // Output INT8 — index 0=open, index 1=closed
    // scale=0.07593961, zero_point=-6
    const float scale = 0.07593961f;
    const int   zp    = -6;

    float openConf   = (out->data.int8[0] - zp) * scale;
    float closedConf = (out->data.int8[1] - zp) * scale;

    if (openConf   < 0.0f) openConf   = 0.0f;
    if (closedConf < 0.0f) closedConf = 0.0f;

    float sum = openConf + closedConf;
    if (sum > 0.0f) {
        openConf   /= sum;
        closedConf /= sum;
    }

    uint8_t state = (closedConf >= openConf) ? EYE_CLOSED : EYE_OPEN;
    uint8_t conf  = (uint8_t)((state == EYE_CLOSED ? closedConf : openConf) * 100.0f);

    InferenceResult res;
    res.state      = state;
    res.confidence = conf;
    res.latencyMs  = latency;
    return res;
}


// ============================================================
// ASCII CONFIDENCE BAR
// ============================================================

void printBar(uint8_t pct) {
    const int W = 10;
    int filled = (pct * W) / 100;
    Serial.print('[');
    for (int i = 0; i < W; i++) Serial.print(i < filled ? '=' : ' ');
    Serial.print(']');
}


// ============================================================
// STATE TRACKING
// ============================================================

static uint32_t closureStartMs    = 0;
static uint32_t closureDurationMs = 0;
static bool     eyeWasClosed      = false;
static uint32_t frameCount        = 0;


// ============================================================
// SETUP
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(2000);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.println();
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║   NeuroMotion – Eye State Tester         ║");
    Serial.println("║   Board: XIAO ESP32-S3 Sense             ║");
    Serial.println("╚══════════════════════════════════════════╝");
    Serial.printf("[SYS] Free PSRAM : %u bytes\n", ESP.getFreePsram());
    Serial.printf("[SYS] Free heap  : %u bytes\n", ESP.getFreeHeap());

    initEspNow();

    if (!initCamera()) {
        Serial.println("[FATAL] Camera init failed — halting.");
        while (true) delay(1000);
    }

    if (!initModel()) {
        Serial.println("[FATAL] Model init failed — halting.");
        while (true) delay(1000);
    }

    Serial.println();
    Serial.println("──────────────────────────────────────────────────────────────────────");
    Serial.println(" State   │ Conf  Bar          │  Duration  │ Latency │ Action");
    Serial.println("──────────────────────────────────────────────────────────────────────");
}


// ============================================================
// LOOP
// ============================================================

void loop() {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("[CAM] Capture failed");
        delay(VISION_LOOP_MS);
        return;
    }

    InferenceResult res = runInference(fb);
    esp_camera_fb_return(fb);
    frameCount++;

    if (res.state == EYE_CLOSED) {
        if (!eyeWasClosed) {
            closureStartMs = millis();
            eyeWasClosed   = true;
        }
        closureDurationMs = millis() - closureStartMs;
    } else {
        eyeWasClosed      = false;
        closureDurationMs = 0;
        closureStartMs    = 0;
    }

    digitalWrite(LED_PIN, res.state == EYE_CLOSED ? HIGH : LOW);

    bool alert = (res.state == EYE_CLOSED) && (closureDurationMs >= MIN_CLOSURE_MS);

    Serial.printf(" %-6s  │ %3d%% ",
        res.state == EYE_CLOSED ? "CLOSED" : "OPEN",
        res.confidence);
    printBar(res.confidence);
    Serial.printf(" │ %6lu ms  │ %4lu ms │ %s\n",
        closureDurationMs,
        res.latencyMs,
        alert ? "<<< ALERT (ESP-NOW SENT)" : "normal");

    if (alert) {
        sendVisionPacket(res.state, res.confidence);
    }

    if (frameCount % 50 == 0) {
        Serial.println("──────────────────────────────────────────────────────────────────────");
        Serial.printf(" Frame: %-6lu  Free heap: %-8u  Free PSRAM: %u\n",
            frameCount, ESP.getFreeHeap(), ESP.getFreePsram());
        Serial.println("──────────────────────────────────────────────────────────────────────");
    }

    delay(VISION_LOOP_MS);
}

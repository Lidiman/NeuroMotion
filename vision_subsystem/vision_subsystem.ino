/*
 * ============================================================
 * NeuroMotion – Vision Subsystem (Dashboard Unit)
 * Board  : Seeed Studio XIAO ESP32-S3 Sense
 * Camera : OV2640 (160x120 Grayscale)
 *
 * Function:
 *   1. Captures 160x120 grayscale frames at full framerate (zero blocking).
 *   2. Preprocesses and runs TFLite Micro INT8 inference.
 *   3. Calculates closed-eye probability score (0 - 100%).
 *   4. Broadcasts VisionPacket_t via ESP-NOW to Decision Engine.
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "esp_camera.h"

#include <Chirale_TensorFlowLite.h>
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "../shared/config.h"
#include "model_data.h"

// ─── Camera Pin Configuration (XIAO ESP32-S3 Sense) ──────────
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

// ─── TFLite Micro Allocations ────────────────────────────────
namespace {
    const tflite::Model*      tfl_model        = nullptr;
    tflite::MicroInterpreter* tfl_interpreter  = nullptr;
    TfLiteTensor*             tfl_input        = nullptr;
    TfLiteTensor*             tfl_output       = nullptr;

    constexpr int kArenaSize = 384 * 1024;
    uint8_t* tensor_arena = nullptr;

    tflite::MicroMutableOpResolver<18> resolver;
}

// ─── ESP-NOW Broadcast State ─────────────────────────────────
static bool espnowReady = false;
static uint8_t broadcastPeerMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    // Non-blocking transmission callback for ESP-IDF v5.x / Arduino Core 3.x
}
#else
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Non-blocking transmission callback for ESP-IDF v4.x
}
#endif

void initEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Initialization failed.");
        return;
    }

    esp_now_register_send_cb(onDataSent);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastPeerMac, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ESP-NOW] Add broadcast peer failed.");
        return;
    }

    espnowReady = true;
    Serial.printf("[ESP-NOW] Ready | MAC: %s\n", WiFi.macAddress().c_str());
}

void broadcastVisionPacket(uint8_t eyeState, uint8_t closedProb, uint16_t latency) {
    if (!espnowReady) return;

    VisionPacket_t packet;
    packet.eyeState      = eyeState;
    packet.closedProbPct = closedProb;
    packet.latencyMs     = latency;
    packet.timestamp     = millis();

    esp_now_send(broadcastPeerMac, (const uint8_t*)&packet, sizeof(VisionPacket_t));
}

// ─── Camera Initialization ───────────────────────────────────
bool initCamera() {
    camera_config_t cfg = {};
    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.ledc_timer   = LEDC_TIMER_0;
    cfg.pin_d0 = Y2_GPIO_NUM; cfg.pin_d1 = Y3_GPIO_NUM;
    cfg.pin_d2 = Y4_GPIO_NUM; cfg.pin_d3 = Y5_GPIO_NUM;
    cfg.pin_d4 = Y6_GPIO_NUM; cfg.pin_d5 = Y7_GPIO_NUM;
    cfg.pin_d6 = Y8_GPIO_NUM; cfg.pin_d7 = Y9_GPIO_NUM;
    cfg.pin_xclk = XCLK_GPIO_NUM; cfg.pin_pclk = PCLK_GPIO_NUM;
    cfg.pin_vsync = VSYNC_GPIO_NUM; cfg.pin_href = HREF_GPIO_NUM;
    cfg.pin_sccb_sda = SIOD_GPIO_NUM; cfg.pin_sccb_scl = SIOC_GPIO_NUM;
    cfg.pin_pwdn = PWDN_GPIO_NUM; cfg.pin_reset = RESET_GPIO_NUM;
    cfg.xclk_freq_hz = 20000000;
    cfg.pixel_format = PIXFORMAT_GRAYSCALE;
    cfg.frame_size   = FRAMESIZE_QQVGA; // 160x120
    cfg.jpeg_quality = 12;
    cfg.fb_count     = 2;
    cfg.grab_mode    = CAMERA_GRAB_LATEST;
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        Serial.printf("[CAM] Init failed with error: 0x%x\n", err);
        return false;
    }
    Serial.println("[CAM] OK - 160x120 Grayscale");
    return true;
}

// ─── TFLite Model Initialization ─────────────────────────────
bool initModel() {
    tensor_arena = (uint8_t*)heap_caps_malloc(kArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!tensor_arena) {
        tensor_arena = (uint8_t*)malloc(kArenaSize);
    }
    if (!tensor_arena) {
        Serial.println("[MODEL] Arena allocation failed.");
        return false;
    }

    tfl_model = tflite::GetModel(g_model);
    if (tfl_model->version() != TFLITE_SCHEMA_VERSION) {
        Serial.printf("[MODEL] Schema mismatch: %d vs %d\n", tfl_model->version(), TFLITE_SCHEMA_VERSION);
        return false;
    }

    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddMaxPool2D();
    resolver.AddMean();
    resolver.AddPad();
    resolver.AddPadV2();
    resolver.AddAdd();
    resolver.AddMul();
    resolver.AddSub();
    resolver.AddReshape();
    resolver.AddFullyConnected();
    resolver.AddSoftmax();
    resolver.AddQuantize();
    resolver.AddDequantize();
    resolver.AddRelu();

    static tflite::MicroInterpreter static_interp(
        tfl_model, resolver, tensor_arena, kArenaSize);
    tfl_interpreter = &static_interp;

    if (tfl_interpreter->AllocateTensors() != kTfLiteOk) {
        Serial.println("[MODEL] AllocateTensors failed.");
        return false;
    }

    tfl_input  = tfl_interpreter->input(0);
    tfl_output = tfl_interpreter->output(0);

    Serial.printf("[MODEL] Ready | In scale=%.5f zp=%d | Out scale=%.5f zp=%d\n",
        tfl_input->params.scale, tfl_input->params.zero_point,
        tfl_output->params.scale, tfl_output->params.zero_point);
    return true;
}

// ─── Fast Preprocessing (Vectorized Math) ─────────────────────
inline void preprocessFrame(const uint8_t* src) {
    int8_t* dst = tfl_input->data.int8;
    const float inv_scale = 1.0f / (255.0f * tfl_input->params.scale);
    const int32_t zp = tfl_input->params.zero_point;
    const int n = MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT;

    for (int i = 0; i < n; i++) {
        dst[i] = (int8_t)constrain((int32_t)(src[i] * inv_scale) + zp, -128, 127);
    }
}

// ─── Setup & Main Loop ────────────────────────────────────────
static uint32_t frameCounter = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.println();
    Serial.println("==================================================");
    Serial.println("  NeuroMotion - Vision Subsystem (ESP32-S3)       ");
    Serial.println("==================================================");

    initEspNow();

    if (!initCamera()) {
        Serial.println("[FATAL] Camera init failed.");
        while (true) delay(1000);
    }

    if (!initModel()) {
        Serial.println("[FATAL] Model init failed.");
        while (true) delay(1000);
    }

    Serial.println("[SYSTEM] Vision pipeline active. Streaming packets...");
}

void loop() {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        return;
    }

    uint32_t t0 = millis();
    preprocessFrame(fb->buf);
    tfl_interpreter->Invoke();
    uint32_t latency = millis() - t0;
    esp_camera_fb_return(fb);

    // Dequantize & Softmax Output
    const float out_scale = tfl_output->params.scale;
    const int32_t out_zp  = tfl_output->params.zero_point;

    float openLogit   = (tfl_output->data.int8[0] - out_zp) * out_scale;
    float closedLogit = (tfl_output->data.int8[1] - out_zp) * out_scale;

    float expOpen   = expf(openLogit);
    float expClosed = expf(closedLogit);
    float closeProb = expClosed / (expOpen + expClosed); // [0.0 - 1.0]

    uint8_t closedPct = (uint8_t)constrain((int)(closeProb * 100.0f), 0, 100);
    uint8_t eyeState  = (closedPct >= 50) ? EYE_CLOSED : EYE_OPEN;

    digitalWrite(LED_PIN, eyeState == EYE_CLOSED ? HIGH : LOW);

    // Seamless non-blocking broadcast to Decision Engine
    broadcastVisionPacket(eyeState, closedPct, (uint16_t)latency);

    frameCounter++;
#if DEBUG_SERIAL
    if (frameCounter % 10 == 0) {
        Serial.printf("[VISION] Frame: %lu | State: %-6s | ClosedProb: %3d%% | Latency: %u ms\n",
            frameCounter, eyeState == EYE_CLOSED ? "CLOSED" : "OPEN", closedPct, latency);
    }
#endif
}

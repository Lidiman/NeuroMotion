/**
 * NeuroMotion – Vision Subsystem
 * Board : Seeed Studio XIAO ESP32-S3 Sense
 * Camera: OV2640 (bundled with XIAO S3 Sense)
 *
 * Function:
 *   - Captures a grayscale frame at ~30 fps.
 *   - Runs NeuroMotionCNN TFLite model (96×96 grayscale → open/closed).
 *   - Sends VisionPacket_t via ESP-NOW to the Decision Engine (ESP32-C3).
 *
 * Dependencies (install via Library Manager or PlatformIO):
 *   - "ESP32 Camera" (espressif/esp32-camera, bundled with ESP32 Arduino core 2.x)
 *   - TensorFlow Lite for Microcontrollers (tflm-esp32 or EloquentTinyML)
 *   - ESP-NOW (built-in, via esp_now.h)
 *
 * Model file:
 *   Place the converted TFLite model as a C byte-array header:
 *     vision_subsystem/model_data.h
 *   Generate it with:
 *     xxd -i neuromotion_model_int8.tflite > model_data.h
 *   Then rename the array to `g_model[]` and its size to `g_model_len`.
 */

#define DEBUG_SERIAL 0

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "model_data.h"          // generated: g_model[], g_model_len
#include "../shared/config.h"
#include "camera_stream.h"   // MJPEG stream + /status endpoint on port 9478

// ─── Camera Pin Map (XIAO ESP32-S3 Sense) ────────────────────────────────────
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

// ─── ESP-NOW Packet ───────────────────────────────────────────────────────────
typedef struct {
    uint8_t  eyeState;      // EYE_OPEN=0, EYE_CLOSED=1
    uint8_t  confidence;    // 0-100 (%)
    uint32_t timestamp;     // millis() since boot
} VisionPacket_t;

static uint8_t decisionMac[] = DECISION_ENGINE_MAC;

// ─── TFLite ───────────────────────────────────────────────────────────────────
// Tensor arena: 96*96*1 input + model overhead. 128 KB is safe for int8 model.
static const int kTensorArenaSize = 48 * 1024;
static uint8_t tensor_arena[kTensorArenaSize];

static tflite::MicroMutableOpResolver<8> resolver;
static const tflite::Model*              model_ptr  = nullptr;
static tflite::MicroInterpreter*         interpreter = nullptr;
static TfLiteTensor*                     input_tensor  = nullptr;
static TfLiteTensor*                     output_tensor = nullptr;

// ─── ESP-NOW Send Callback ────────────────────────────────────────────────────
void onDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
#if DEBUG_SERIAL
    Serial.print("[ESP-NOW] Send: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
#endif
}

// ─── Camera Init ─────────────────────────────────────────────────────────────
bool initCamera() {
    camera_config_t cfg;
    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.ledc_timer   = LEDC_TIMER_0;
    cfg.pin_d0  = Y2_GPIO_NUM;
    cfg.pin_d1  = Y3_GPIO_NUM;
    cfg.pin_d2  = Y4_GPIO_NUM;
    cfg.pin_d3  = Y5_GPIO_NUM;
    cfg.pin_d4  = Y6_GPIO_NUM;
    cfg.pin_d5  = Y7_GPIO_NUM;
    cfg.pin_d6  = Y8_GPIO_NUM;
    cfg.pin_d7  = Y9_GPIO_NUM;
    cfg.pin_xclk  = XCLK_GPIO_NUM;
    cfg.pin_pclk  = PCLK_GPIO_NUM;
    cfg.pin_vsync = VSYNC_GPIO_NUM;
    cfg.pin_href  = HREF_GPIO_NUM;
    cfg.pin_sscb_sda = SIOD_GPIO_NUM;
    cfg.pin_sscb_scl = SIOC_GPIO_NUM;
    cfg.pin_pwdn  = PWDN_GPIO_NUM;
    cfg.pin_reset = RESET_GPIO_NUM;
    cfg.xclk_freq_hz = 20000000;
    cfg.pixel_format = PIXFORMAT_GRAYSCALE;  // grayscale → direct to model
    cfg.frame_size   = FRAMESIZE_96X96;      // native 96x96, no resize needed
    cfg.jpeg_quality = 12;
    cfg.fb_count     = 1;
    cfg.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
    cfg.fb_location  = CAMERA_FB_IN_DRAM;   // use DRAM for frame buffer

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        Serial.printf("[CAM] Init failed: 0x%x\n", err);
        return false;
    }
    Serial.println("[CAM] Init OK");
    return true;
}

// ─── TFLite Init ─────────────────────────────────────────────────────────────
bool initModel() {
    model_ptr = tflite::GetModel(g_model);
    if (model_ptr->version() != TFLITE_SCHEMA_VERSION) {
        Serial.println("[TFLite] Schema version mismatch!");
        return false;
    }

    // Register only needed ops (keeps binary smaller)
    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddMaxPool2D();
    resolver.AddReshape();
    resolver.AddFullyConnected();
    resolver.AddSoftmax();

    static tflite::MicroInterpreter static_interpreter(
        model_ptr, resolver, tensor_arena, kTensorArenaSize);
    interpreter = &static_interpreter;

    if (interpreter->AllocateTensors() != kTfLiteOk) {
        Serial.println("[TFLite] AllocateTensors failed!");
        return false;
    }

    input_tensor  = interpreter->input(0);
    output_tensor = interpreter->output(0);

    Serial.printf("[TFLite] Input:  [%d,%d,%d,%d] type=%d\n",
        input_tensor->dims->data[0], input_tensor->dims->data[1],
        input_tensor->dims->data[2], input_tensor->dims->data[3],
        input_tensor->type);
    Serial.printf("[TFLite] Output: [%d,%d] type=%d\n",
        output_tensor->dims->data[0], output_tensor->dims->data[1],
        output_tensor->type);
    Serial.println("[TFLite] Init OK");
    return true;
}

// ─── Inference ───────────────────────────────────────────────────────────────
/**
 * Run inference on a 96x96 grayscale frame.
 * @param  pixels   Pointer to grayscale pixel data (96*96 bytes).
 * @param  eyeState Output: EYE_OPEN or EYE_CLOSED.
 * @param  conf     Output: confidence 0-100.
 * @return true on success.
 */
bool runInference(const uint8_t* pixels, uint8_t& eyeState, uint8_t& conf) {
    // Copy pixels into model input, normalized to int8 range
    // Using quantization params embedded in model_data.h
    int8_t* inp = input_tensor->data.int8;
    float input_scale = MODEL_INPUT_SCALE;
    int32_t input_zp  = MODEL_INPUT_ZP;
    for (int i = 0; i < MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT; i++) {
        float normalized = (float)pixels[i] / 255.0f;  // [0,1]
        inp[i] = (int8_t)((int32_t)(normalized / input_scale) + input_zp);
    }

    if (interpreter->Invoke() != kTfLiteOk) {
        Serial.println("[TFLite] Invoke failed!");
        return false;
    }

    // Output: [open_score, closed_score] — int8 quantized
    // Dequantize using output tensor's scale + zero_point
    float output_scale = output_tensor->params.scale;
    int32_t output_zp  = output_tensor->params.zero_point;
    int8_t* out        = output_tensor->data.int8;
    float openConf  = output_scale * (out[EYE_OPEN]   - output_zp);
    float closedConf= output_scale * (out[EYE_CLOSED] - output_zp);

#if DEBUG_SERIAL
    Serial.printf("[INF] open=%.3f closed=%.3f\n", openConf, closedConf);
#endif

    eyeState = (closedConf > openConf) ? EYE_CLOSED : EYE_OPEN;
    conf     = (uint8_t)(((eyeState == EYE_CLOSED) ? closedConf : openConf) * 100.0f);
    conf     = (conf > 100) ? 100 : conf;
    return true;
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[NeuroMotion] Vision Subsystem booting...");

    // Camera
    if (!initCamera()) {
        Serial.println("[FATAL] Camera init failed. Halting.");
        while (1) delay(1000);
    }

    // TFLite model
    if (!initModel()) {
        Serial.println("[FATAL] Model init failed. Halting.");
        while (1) delay(1000);
    }

    // Wi-Fi in STA mode (required for ESP-NOW)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    Serial.print("[WIFI] MAC: ");
    Serial.println(WiFi.macAddress());

    // ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[FATAL] ESP-NOW init failed. Halting.");
        while (1) delay(1000);
    }
    esp_now_register_send_cb(onDataSent);

    // Start camera stream server (port 9478)
    String camIP = streamServerInit();
    Serial.printf("[STREAM] Camera accessible at http://%s:%d/\n", camIP.c_str(), STREAM_PORT);

    // Register Decision Engine as peer
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, decisionMac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("[WARN] Failed to add Decision Engine peer — check MAC.");
    }

    Serial.println("[NeuroMotion] Vision Subsystem ready.");
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
    unsigned long frameStart = millis();

    // Capture frame
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("[CAM] Frame capture failed");
        delay(VISION_LOOP_MS);
        return;
    }

    // Sanity check: expect 96*96 greyscale bytes
    if (fb->len != (MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT)) {
        Serial.printf("[CAM] Unexpected frame size: %d\n", fb->len);
        esp_camera_fb_return(fb);
        delay(VISION_LOOP_MS);
        return;
    }

    // Run inference
    uint8_t eyeState = EYE_OPEN;
    uint8_t conf     = 0;
    bool ok = runInference(fb->buf, eyeState, conf);
    esp_camera_fb_return(fb);

    if (!ok) {
        delay(VISION_LOOP_MS);
        return;
    }

    // Update stream status endpoint
    streamUpdateState(eyeState, conf, (float)conf / 100.0f);

    // Build and send ESP-NOW packet
    VisionPacket_t pkt;
    pkt.eyeState  = eyeState;
    pkt.confidence= conf;
    pkt.timestamp = millis();

    esp_now_send(decisionMac, (uint8_t*)&pkt, sizeof(pkt));

    // Maintain ~30 fps
    unsigned long elapsed = millis() - frameStart;
    if (elapsed < VISION_LOOP_MS) {
        delay(VISION_LOOP_MS - elapsed);
    }

    // Handle HTTP clients (streaming)
    streamServerLoop();
}

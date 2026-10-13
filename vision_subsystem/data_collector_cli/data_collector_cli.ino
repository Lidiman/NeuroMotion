/*
 * ============================================================
 * NeuroMotion – Color Dataset Collector (QQVGA 160x120 RGB565)
 * Board  : Seeed Studio XIAO ESP32-S3 Sense
 * Camera : OV2640 (160x120 Color RGB565)
 *
 * Stream Protocol:
 *   [0xAA 0x55 0xAA 0x55] (4 bytes Sync Header)
 *   [Length 4 bytes: 32-bit big-endian] -> 160 * 120 * 2 = 38400 bytes
 *   [Raw RGB565 Payload: 38400 bytes]
 * ============================================================
 */

#include <Arduino.h>
#include "esp_camera.h"

// ─── Pin Definition XIAO ESP32-S3 Sense ──────────────────────
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

const uint8_t SYNC_HEADER[4] = {0xAA, 0x55, 0xAA, 0x55};

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
    cfg.pixel_format = PIXFORMAT_JPEG;
    cfg.frame_size   = FRAMESIZE_QQVGA; // 160x120
    cfg.jpeg_quality = 10;              // 0-63 lower means higher quality
    cfg.fb_count     = 2;
    cfg.grab_mode    = CAMERA_GRAB_LATEST;
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        Serial.printf("[CAM] Init failed: 0x%x\n", err);
        return false;
    }
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(1500);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    if (!initCamera()) {
        while (1) {
            digitalWrite(LED_PIN, HIGH);
            delay(200);
            digitalWrite(LED_PIN, LOW);
            delay(200);
        }
    }
}

void loop() {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
        Serial.write(SYNC_HEADER, 4);
        uint32_t len = fb->len;
        Serial.write((uint8_t)((len >> 24) & 0xFF));
        Serial.write((uint8_t)((len >> 16) & 0xFF));
        Serial.write((uint8_t)((len >> 8) & 0xFF));
        Serial.write((uint8_t)(len & 0xFF));
        Serial.write(fb->buf, fb->len);
        esp_camera_fb_return(fb);
    }
    delay(30);
}

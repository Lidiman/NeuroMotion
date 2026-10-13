/*
 * ============================================================
 * NeuroMotion – Vision Subsystem: Dataset Collector & Streamer
 * Board  : Seeed Studio XIAO ESP32-S3 Sense
 * Camera : OV2640 (96x96 Grayscale)
 *
 * Features:
 *   1. Connects to Wi-Fi (STA) or creates AP if unavailable.
 *   2. Serves Live Browser Web Interface at http://<IP>:80/
 *   3. Endpoints:
 *      - GET /        : Interactive Web UI with Live View & Capture Controls
 *      - GET /stream  : Real-time Grayscale stream (multipart)
 *      - GET /snap    : Single 96x96 raw binary/PGM/JPEG frame
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"

// ─── Wi-Fi Configuration ──────────────────────────────────────
// Set your Wi-Fi credentials here to access via local network IP.
// If it fails to connect within 10s, it falls back to AP mode.
const char* STA_SSID = "Haloo";
const char* STA_PASS = "HhAaLlOoOo";

const char* AP_SSID  = "NeuroMotion-Collector";
const char* AP_PASS  = "neuromotion123";

// ─── Camera Pin Mapping (XIAO ESP32-S3 Sense) ─────────────────
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

WebServer server(80);

// ─── Web Interface HTML ───────────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>NeuroMotion Data Collector</title>
    <style>
        * { box-sizing: border-box; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, monospace; }
        body { background: #0f172a; color: #f8fafc; margin: 0; padding: 20px; display: flex; flex-direction: column; align-items: center; }
        .card { background: #1e293b; padding: 24px; border-radius: 12px; width: 100%; max-width: 520px; box-shadow: 0 10px 25px rgba(0,0,0,0.5); }
        h2 { margin-top: 0; color: #38bdf8; font-size: 1.4rem; text-align: center; }
        .preview-box { display: flex; justify-content: center; margin: 16px 0; background: #000; border-radius: 8px; padding: 10px; border: 2px solid #334155; }
        canvas { width: 288px; height: 288px; image-rendering: pixelated; border-radius: 4px; }
        .row { display: flex; gap: 10px; margin-bottom: 12px; }
        .field { flex: 1; display: flex; flex-direction: column; }
        label { font-size: 0.8rem; color: #94a3b8; margin-bottom: 4px; }
        input, select, button { padding: 10px 12px; border-radius: 6px; border: 1px solid #475569; background: #0f172a; color: #fff; font-size: 0.9rem; }
        input:focus, select:focus { outline: 2px solid #38bdf8; }
        button { cursor: pointer; font-weight: bold; border: none; transition: 0.2s; }
        button.start { background: #10b981; color: white; }
        button.start:hover { background: #059669; }
        button.stop { background: #ef4444; color: white; }
        button.stop:hover { background: #dc2626; }
        .stats { margin-top: 14px; background: #0f172a; border-radius: 6px; padding: 12px; font-size: 0.85rem; border: 1px solid #334155; }
        .stat-item { display: flex; justify-content: space-between; margin-bottom: 4px; }
        .badge { background: #38bdf8; color: #0f172a; padding: 2px 6px; border-radius: 4px; font-weight: bold; }
    </style>
</head>
<body>
    <div class="card">
        <h2>📷 NeuroMotion Collector (96x96)</h2>
        
        <div class="preview-box">
            <canvas id="viewCanvas" width="96" height="96"></canvas>
        </div>

        <div class="row">
            <div class="field">
                <label>Target Folder Label</label>
                <select id="folderLabel">
                    <option value="noglasses_open">noglasses_open</option>
                    <option value="noglasses_close">noglasses_close</option>
                    <option value="glasses_open">glasses_open</option>
                    <option value="glasses_close">glasses_close</option>
                </select>
            </div>
            <div class="field">
                <label>Capture Interval (ms)</label>
                <input type="number" id="intervalMs" value="50" min="20" max="5000">
            </div>
        </div>

        <div class="row">
            <div class="field">
                <label>Prefix / Subject ID</label>
                <input type="text" id="filePrefix" value="xavier_s1">
            </div>
        </div>

        <div class="row" style="margin-top: 8px;">
            <button id="toggleBtn" class="start" style="width: 100%;" onclick="toggleRecording()">▶ Start Auto Download Samples</button>
        </div>

        <div class="stats">
            <div class="stat-item"><span>Status:</span><span id="recStatus" class="badge">IDLE</span></div>
            <div class="stat-item"><span>Captured Frames:</span><strong id="frameCounter">0</strong></div>
            <div class="stat-item"><span>FPS:</span><span id="fpsDisplay">0 fps</span></div>
        </div>
    </div>

    <script>
        const canvas = document.getElementById('viewCanvas');
        const ctx = canvas.getContext('2d');
        const imgData = ctx.createImageData(96, 96);

        let recording = false;
        let frameCount = 0;
        let lastFrameTime = performance.now();
        let fps = 0;

        async function fetchFrame() {
            try {
                const t0 = performance.now();
                const res = await fetch('/raw', { cache: 'no-store' });
                if (!res.ok) throw new Error('Network error');
                const buf = await res.arrayBuffer();
                const bytes = new Uint8Array(buf);

                if (bytes.length === 96 * 96) {
                    for (let i = 0; i < bytes.length; i++) {
                        const val = bytes[i];
                        const p = i * 4;
                        imgData.data[p]     = val;
                        imgData.data[p + 1] = val;
                        imgData.data[p + 2] = val;
                        imgData.data[p + 3] = 255;
                    }
                    ctx.putImageData(imgData, 0, 0);

                    const now = performance.now();
                    fps = Math.round(1000 / (now - lastFrameTime));
                    lastFrameTime = now;
                    document.getElementById('fpsDisplay').innerText = fps + ' fps';

                    if (recording) {
                        saveCurrentFrame();
                    }
                }
            } catch (err) {
                console.error(err);
            }

            const interval = recording ? parseInt(document.getElementById('intervalMs').value) : 40;
            setTimeout(fetchFrame, interval);
        }

        function toggleRecording() {
            recording = !recording;
            const btn = document.getElementById('toggleBtn');
            const st = document.getElementById('recStatus');
            if (recording) {
                btn.className = 'stop';
                btn.innerText = '⏹ Stop Recording';
                st.innerText = 'RECORDING';
                st.style.background = '#ef4444';
                st.style.color = '#fff';
            } else {
                btn.className = 'start';
                btn.innerText = '▶ Start Auto Download Samples';
                st.innerText = 'IDLE';
                st.style.background = '#38bdf8';
                st.style.color = '#0f172a';
            }
        }

        function saveCurrentFrame() {
            frameCount++;
            document.getElementById('frameCounter').innerText = frameCount;

            const folder = document.getElementById('folderLabel').value;
            const prefix = document.getElementById('filePrefix').value;
            const timestamp = Date.now();
            const filename = `${folder}_${prefix}_${String(frameCount).padStart(5, '0')}_${timestamp}.jpg`;

            canvas.toBlob(blob => {
                const a = document.createElement('a');
                a.href = URL.createObjectURL(blob);
                a.download = filename;
                a.click();
                URL.revokeObjectURL(a.href);
            }, 'image/jpeg', 0.95);
        }

        // Start render loop
        fetchFrame();
    </script>
</body>
</html>
)rawhtml";

// ─── Camera Setup ─────────────────────────────────────────────
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
    cfg.frame_size   = FRAMESIZE_96X96;
    cfg.jpeg_quality = 12;
    cfg.fb_count     = 2;
    cfg.grab_mode    = CAMERA_GRAB_LATEST;
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        Serial.printf("[CAM] Init failed with error: 0x%x\n", err);
        return false;
    }
    return true;
}

// ─── HTTP Endpoints ───────────────────────────────────────────
void handleRoot() {
    server.send_P(200, "text/html", INDEX_HTML);
}

void handleRawFrame() {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        server.send(503, "text/plain", "Camera Busy");
        return;
    }
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.send_P(200, "application/octet-stream", (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.println();
    Serial.println("==============================================");
    Serial.println("  NeuroMotion - OV2640 Web Data Collector     ");
    Serial.println("==============================================");

    if (!initCamera()) {
        Serial.println("[ERROR] Camera Init Failed!");
        while (1) { delay(1000); }
    }
    Serial.println("[OK] Camera Initialized (96x96 Grayscale)");

    // Wi-Fi Connection
    WiFi.mode(WIFI_STA);
    WiFi.begin(STA_SSID, STA_PASS);
    Serial.printf("[WIFI] Connecting to %s", STA_SSID);

    int timeout = 20;
    while (WiFi.status() != WL_CONNECTED && timeout > 0) {
        delay(500);
        Serial.print(".");
        timeout--;
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("[WIFI] Connected Successfully!");
        Serial.print("[WIFI] Web UI Address : http://");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("[WIFI] Connection failed. Starting Access Point mode...");
        WiFi.mode(WIFI_AP);
        WiFi.softAP(AP_SSID, AP_PASS);
        Serial.printf("[AP] SSID: %s | PASS: %s\n", AP_SSID, AP_PASS);
        Serial.print("[AP] Web UI Address : http://");
        Serial.println(WiFi.softAPIP());
    }

    server.on("/", HTTP_GET, handleRoot);
    server.on("/raw", HTTP_GET, handleRawFrame);
    server.begin();
    Serial.println("[HTTP] Server ready on port 80");
}

void loop() {
    server.handleClient();
}

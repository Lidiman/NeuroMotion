/**
 * NeuroMotion – Camera MJPEG Streaming Server
 * Runs on XIAO ESP32-S3, port 9478.
 *
 * Access: http://<ESP_IP>:9478/stream  → MJPEG live feed
 *         http://<ESP_IP>:9478/snap    → single JPEG snapshot
 *         http://<ESP_IP>:9478/status  → JSON: eyeState, confidence, riskScore
 *
 * NOTE: Wi-Fi AP mode is used here so the ESP32-S3 is always reachable
 * even without a router. Connect to SSID "NeuroMotion-Cam".
 *
 * Alternatively, set STREAM_WIFI_MODE to WIFI_STA and fill credentials
 * to join your existing network.
 */

#ifndef CAMERA_STREAM_H
#define CAMERA_STREAM_H

#include <WebServer.h>
#include "esp_camera.h"

// ─── Wi-Fi Config ─────────────────────────────────────────────────────────────
#define STREAM_WIFI_MODE   WIFI_AP      // use WIFI_AP for standalone, WIFI_STA to join router
#define STREAM_AP_SSID     "NeuroMotion-Cam"
#define STREAM_AP_PASS     "neuromotion"   // min 8 chars
#define STREAM_STA_SSID    "YOUR_SSID"     // fill if using STA mode
#define STREAM_STA_PASS    "YOUR_PASS"
#define STREAM_PORT        9478

// ─── Shared state (written by main loop, read by HTTP handlers) ───────────────
static volatile uint8_t  g_eyeState   = 0;
static volatile uint8_t  g_confidence = 0;
static volatile float    g_riskScore  = 0.0f;

static WebServer httpServer(STREAM_PORT);

// ─── Handlers ─────────────────────────────────────────────────────────────────
static void handleStream() {
    // MJPEG multipart stream
    WiFiClient client = httpServer.client();
    String boundary = "neuromotion_frame";
    httpServer.sendHeader("Content-Type", "multipart/x-mixed-replace; boundary=" + boundary);
    httpServer.sendHeader("Access-Control-Allow-Origin", "*");
    httpServer.sendHeader("Cache-Control", "no-cache");
    httpServer.send(200);

    while (client.connected()) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) { delay(10); continue; }

        // For streaming we want JPEG — switch pixel format temporarily
        // or use esp_camera_fb_get with JPEG format (set in setup if streaming enabled)
        // Here we assume FRAMESIZE_96X96 + PIXFORMAT_JPEG for stream
        client.print("--" + boundary + "\r\n");
        client.print("Content-Type: image/jpeg\r\n");
        client.print("Content-Length: " + String(fb->len) + "\r\n\r\n");
        client.write(fb->buf, fb->len);
        client.print("\r\n");
        esp_camera_fb_return(fb);
        delay(33); // ~30 fps
    }
}

static void handleSnap() {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { httpServer.send(503, "text/plain", "Camera not ready"); return; }
    httpServer.sendHeader("Access-Control-Allow-Origin", "*");
    httpServer.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
}

static void handleStatus() {
    char json[128];
    snprintf(json, sizeof(json),
        "{\"eyeState\":%d,\"eyeLabel\":\"%s\",\"confidence\":%d,\"riskScore\":%.3f}",
        (int)g_eyeState,
        g_eyeState == 1 ? "closed" : "open",
        (int)g_confidence,
        (float)g_riskScore
    );
    httpServer.sendHeader("Access-Control-Allow-Origin", "*");
    httpServer.send(200, "application/json", json);
}

static void handleRoot() {
    String html = R"rawhtml(
<!DOCTYPE html><html><head>
<meta charset="utf-8">
<title>NeuroMotion Camera</title>
<style>
  body { background:#111; color:#eee; font-family:monospace; text-align:center; padding:20px; }
  img  { max-width:480px; border:2px solid #444; }
  #status { margin-top:12px; font-size:1.2em; }
</style>
</head><body>
<h2>NeuroMotion Live Feed</h2>
<img src="/stream" id="feed">
<div id="status">Loading...</div>
<script>
  setInterval(async () => {
    const r = await fetch('/status');
    const d = await r.json();
    document.getElementById('status').innerHTML =
      'Eye: <b>' + d.eyeLabel.toUpperCase() + '</b> | Conf: ' + d.confidence + '% | Risk: ' + d.riskScore.toFixed(3);
  }, 500);
</script>
</body></html>
)rawhtml";
    httpServer.send(200, "text/html", html);
}

// ─── Public API ───────────────────────────────────────────────────────────────

/**
 * Call once in setup(), AFTER esp_now_init().
 * Returns the IP address to print to Serial.
 */
String streamServerInit() {
    String ip;

#if STREAM_WIFI_MODE == WIFI_AP
    // Note: ESP-NOW requires STA mode; we run dual mode (AP+STA)
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(STREAM_AP_SSID, STREAM_AP_PASS);
    ip = WiFi.softAPIP().toString();
    Serial.printf("[STREAM] AP started: SSID=%s IP=%s\n", STREAM_AP_SSID, ip.c_str());
#else
    WiFi.mode(WIFI_STA);
    WiFi.begin(STREAM_STA_SSID, STREAM_STA_PASS);
    Serial.print("[STREAM] Connecting to WiFi");
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) {
        delay(500); Serial.print(".");
    }
    ip = WiFi.localIP().toString();
    Serial.printf("\n[STREAM] Connected, IP=%s\n", ip.c_str());
#endif

    httpServer.on("/",       HTTP_GET, handleRoot);
    httpServer.on("/stream", HTTP_GET, handleStream);
    httpServer.on("/snap",   HTTP_GET, handleSnap);
    httpServer.on("/status", HTTP_GET, handleStatus);
    httpServer.begin();

    Serial.printf("[STREAM] HTTP server on port %d\n", STREAM_PORT);
    Serial.printf("[STREAM] Dashboard: http://%s:%d/\n", ip.c_str(), STREAM_PORT);
    return ip;
}

/** Call every loop() iteration. */
void streamServerLoop() {
    httpServer.handleClient();
}

/** Update inference result so /status endpoint reflects latest data. */
void streamUpdateState(uint8_t eyeState, uint8_t conf, float risk) {
    g_eyeState   = eyeState;
    g_confidence = conf;
    g_riskScore  = risk;
}

#endif // CAMERA_STREAM_H

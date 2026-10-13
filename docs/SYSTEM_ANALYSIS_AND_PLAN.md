# NeuroMotion: Sistem Deteksi Kantuk & Microsleep Pengendara Motor

---

## 1. Threshold GY-25 untuk Deteksi Kantuk

Sensor GY-25 dipasang di helm untuk mengukur orientasi kepala:
- **Pitch**: Gerakan menunduk (nodding) atau mendongak.
- **Roll**: Kemiringan kepala ke kiri atau kanan (head tilt).
- **Yaw**: Menoleh ke kiri atau kanan.

### Tabel Nilai Threshold Optimal

| Parameter Orientasi | Rentang Berkendara Normal | Threshold Kantuk / Microsleep | Durasi Bertahan |
|---|---|---|---|
| Pitch (Kepala Menunduk) | -10 deg s.d. +15 deg | Pitch <= -20 deg | >= 1.0 detik |
| Roll (Kepala Miring) | -12 deg s.d. +12 deg | abs(Roll) >= 25 deg | >= 1.0 detik |
| Variansi Gerakan | Var > 5.0 (gerak aktif jalan) | Var <= 1.5 (kepala diam kaku/stupor) | >= 2.0 detik (saat mata terpejam) |
| Sentakan Bangun (Jerk) | Halus | delta_Pitch > 25 deg / 300 ms | Instant flag (kaget bangun) |

Catatan Penting:
Gunakan kalibrasi baseline (Offset) saat sistem pertama kali menyala:
delta_Pitch = Pitch_saat_ini - Pitch_baseline
delta_Roll = Roll_saat_ini - Roll_baseline

---

## 2. Analisis Struktur Direktori Proyek

- **shared/**:
  - config.h: Definisi konstanta sistem, MAC Address ESP-NOW, bobot fusi, dan konfigurasi pin.
- **vision_subsystem/** (Dashboard Unit: XIAO ESP32-S3 Sense):
  - vision_subsystem.ino: Program inferensi kamera OV2640 + TFLite Micro INT8 + Pengirim ESP-NOW.
  - model_data.h: Array C-byte model INT8 hasil training.
  - data_collector_cli/: Tool pengambil dataset real-time dari kamera hardware.
- **decision_engine/** (Helmet Unit: ESP32-C3 Super Mini + GY-25):
  - decision_engine.ino: Penerima ESP-NOW + Reader GY-25 UART + Algoritma Fusi 5-10 Window + Driver Buzzer & Vibration Motor.

---

## 3. Rencana Implementasi Tiap Hardware

### A. Vision Subsystem (XIAO ESP32-S3 Sense - Box Dashboard)
1. Inisialisasi kamera OV2640 pada resolusi 160x120 Grayscale.
2. Inferensi model TFLite Micro INT8 pada frame kamera.
3. Ekstraksi confidence score probabilitas mata tertutup (0% - 100%).
4. Mengirimkan VisionPacket_t via ESP-NOW tanpa jeda delay (zero-delay loop) ke ESP32-C3.

### B. Motion Subsystem (GY-25 - Box Helm)
1. Komunikasi UART 9600 baud dengan ESP32-C3 Super Mini.
2. Output data sudut: Yaw, Pitch, Roll dengan frekuensi 20 Hz.
3. Mengirimkan nilai deviasi sudut relatif terhadap posisi tegak normal pengendara.

### C. Decision Engine (ESP32-C3 Super Mini - Box Helm)
1. **Sliding Window Buffer**:
   - Buffer Vision: Menyimpan rata-rata 5 - 10 frame terakhir data mata.
   - Buffer Motion: Menyimpan rata-rata 5 - 10 sampel terakhir data sudut GY-25.
2. **Formula Fusi Keputusan (Multi-Modal Fusion)**:
   Skor_Akhir = (W_Visual * Prob_Mata_Tertutup) + (W_Durasi * Skor_Durasi) + (W_Gerak * Skor_Kemiringan)
   - W_Visual = 0.45
   - W_Durasi = 0.35
   - W_Gerak  = 0.20
3. **Logika Output Aktuator**:
   - Skor_Akhir <= 50%: Status NORMAL (Buzzer & Haptic Motor OFF).
   - Skor_Akhir > 50%: Status DROWSY / MICROSLEEP (Buzzer 2 kHz pulsed + Haptic Motor aktif).

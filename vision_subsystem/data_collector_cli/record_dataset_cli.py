"""
NeuroMotion - Interactive CLI Colored Dataset Collector
Receives RGB565 stream from ESP32-S3 over Serial and saves 96x96 colored images (RGB/BGR).

Usage:
    python record_dataset_cli.py
"""

import os
import sys
import time
import serial
import serial.tools.list_ports
import numpy as np
import cv2

DEFAULT_BASE_DIR = r"C:\Users\Xavier\Documents\Kaggle Projects (DA & DS)\Neuromotion (Drowsiness Detection)\ov2640_samples"
SUBDIRS = [
    "noglasses_open",
    "noglasses_close",
    "glasses_open",
    "glasses_close"
]
SYNC_HEADER = bytes([0xAA, 0x55, 0xAA, 0x55])

def select_serial_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("[Error] No COM ports detected! Please connect your XIAO ESP32-S3.")
        sys.exit(1)
    
    print("\nAvailable Serial Ports:")
    for idx, p in enumerate(ports):
        print(f"  [{idx + 1}] {p.device} - {p.description}")
    
    while True:
        choice = input(f"\nSelect COM Port (1-{len(ports)}) [Default: 1]: ").strip()
        if choice == "":
            return ports[0].device
        if choice.isdigit() and 1 <= int(choice) <= len(ports):
            return ports[int(choice) - 1].device
        print("Invalid choice, please re-enter.")

def select_target_directory():
    print("\n" + "=" * 60)
    print(" Select Dataset Category:")
    print("=" * 60)
    for idx, name in enumerate(SUBDIRS):
        dest_path = os.path.join(DEFAULT_BASE_DIR, name)
        existing_count = len([f for f in os.listdir(dest_path) if f.endswith(('.jpg', '.png'))]) if os.path.exists(dest_path) else 0
        print(f"  [{idx + 1}] {name.ljust(18)} (Current samples: {existing_count})")
    
    while True:
        choice = input(f"\nChoose category (1-4): ").strip()
        if choice.isdigit() and 1 <= int(choice) <= 4:
            selected_sub = SUBDIRS[int(choice) - 1]
            break
        print("Invalid choice. Please select 1, 2, 3, or 4.")
    
    prefix = input("Enter Subject/Session Prefix [e.g. xavier]: ").strip()
    if not prefix:
        prefix = "sample"

    full_target_path = os.path.join(DEFAULT_BASE_DIR, selected_sub)
    os.makedirs(full_target_path, exist_ok=True)
    return selected_sub, full_target_path, prefix

def main():
    print("=" * 60)
    print(" NeuroMotion - Fast Live Preview & Dataset Collector")
    print("=" * 60)

    port = select_serial_port()
    selected_sub, target_dir, prefix = select_target_directory()

    print("\n" + "-" * 60)
    print(f" Port        : {port}")
    print(f" Target Dir  : {target_dir}")
    print(f" File Prefix : {prefix}")
    print(" Action      : Saving frames with Smooth Live Preview.")
    print(" Press 'q' or ESC in preview window (or Ctrl+C) to STOP.")
    print("-" * 60 + "\n")

    input("Press [ENTER] to start recording...")

    try:
        ser = serial.Serial(port, 115200, timeout=1.0)
        ser.set_buffer_size(rx_size=128000)
        ser.reset_input_buffer()
    except Exception as e:
        print(f"[Error] Failed to open {port}: {e}")
        sys.exit(1)

    window_name = "NeuroMotion Live View"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 320, 240)

    saved_count = 0
    t_start = time.time()
    last_gui_update = 0

    try:
        while True:
            header_bytes = ser.read_until(SYNC_HEADER)
            if not header_bytes.endswith(SYNC_HEADER):
                continue

            len_bytes = ser.read(4)
            if len(len_bytes) != 4:
                continue
            payload_len = (len_bytes[0] << 24) | (len_bytes[1] << 16) | (len_bytes[2] << 8) | len_bytes[3]

            if payload_len <= 0 or payload_len > 100000:
                continue

            jpeg_bytes = ser.read(payload_len)
            if len(jpeg_bytes) != payload_len:
                continue

            saved_count += 1
            timestamp_ms = int(time.time() * 1000)
            filename = f"{selected_sub}_{prefix}_{saved_count:05d}_{timestamp_ms}.jpg"
            save_path = os.path.join(target_dir, filename)
            with open(save_path, "wb") as f:
                f.write(jpeg_bytes)

            elapsed = time.time() - t_start
            fps = saved_count / max(0.001, elapsed)

            print(f"[{saved_count:05d}] Saved -> {selected_sub}/{filename} ({fps:.1f} FPS, {payload_len/1024:.1f} KB)")
            sys.stdout.flush()

            now = time.time()
            if now - last_gui_update >= 0.05:
                last_gui_update = now
                nparr = np.frombuffer(jpeg_bytes, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                if frame is not None:
                    display = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_NEAREST)
                    cv2.putText(display, f"REC: {saved_count}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(display, f"FPS: {fps:.1f}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    cv2.imshow(window_name, display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break

    except KeyboardInterrupt:
        print("\n\n[INFO] Stopped by user (Ctrl+C).")
    finally:
        ser.close()
        cv2.destroyAllWindows()
        print(f"\n[DONE] Total {saved_count} color images saved to:\n       -> {target_dir}\n")

if __name__ == "__main__":
    main()

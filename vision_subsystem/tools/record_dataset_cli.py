"""
NeuroMotion - Pure Headless CLI Colored Dataset Collector
Receives RGB565 stream from ESP32-S3 over Serial and saves 96x96 colored images (RGB/BGR)
directly to target folders with immediate terminal logging per frame.

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
FRAME_PAYLOAD_SIZE = 96 * 96 * 2  # RGB565 16-bit per pixel = 18432 bytes

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
    print(" NeuroMotion - Headless Colored 96x96 Dataset Collector")
    print("=" * 60)

    port = select_serial_port()
    selected_sub, target_dir, prefix = select_target_directory()

    print("\n" + "-" * 60)
    print(f" Port        : {port}")
    print(f" Target Dir  : {target_dir}")
    print(f" File Prefix : {prefix}")
    print(" Action      : Continuously saving 96x96 COLOR images.")
    print(" Press Ctrl+C to STOP recording.")
    print("-" * 60 + "\n")

    input("Press [ENTER] to start recording...")

    try:
        ser = serial.Serial(port, 115200, timeout=1.0)
        ser.reset_input_buffer()
    except Exception as e:
        print(f"[Error] Failed to open {port}: {e}")
        sys.exit(1)

    saved_count = 0
    t_start = time.time()

    try:
        while True:
            # Sync to header
            header_bytes = ser.read_until(SYNC_HEADER)
            if not header_bytes.endswith(SYNC_HEADER):
                continue

            # Read 2-byte length
            len_bytes = ser.read(2)
            if len(len_bytes) != 2:
                continue
            payload_len = (len_bytes[0] << 8) | len_bytes[1]

            if payload_len != FRAME_PAYLOAD_SIZE:
                continue

            # Read raw RGB565 buffer
            raw_rgb565 = ser.read(payload_len)
            if len(raw_rgb565) != FRAME_PAYLOAD_SIZE:
                continue

            arr = np.frombuffer(raw_rgb565, dtype=np.uint8).reshape((96, 96, 2))
            high = arr[:, :, 0].astype(np.uint16)
            low = arr[:, :, 1].astype(np.uint16)
            pixel565 = (high << 8) | low

            r = ((pixel565 >> 11) & 0x1F) * 255 // 31
            g = ((pixel565 >> 5) & 0x3F) * 255 // 63
            b = (pixel565 & 0x1F) * 255 // 31
            bgr_img = np.dstack((b, g, r)).astype(np.uint8)

            # Save 96x96 color JPEG
            saved_count += 1
            timestamp_ms = int(time.time() * 1000)
            filename = f"{selected_sub}_{prefix}_{saved_count:05d}_{timestamp_ms}.jpg"
            save_path = os.path.join(target_dir, filename)
            cv2.imwrite(save_path, bgr_img, [int(cv2.IMWRITE_JPEG_QUALITY), 95])

            elapsed = time.time() - t_start
            fps = saved_count / max(0.001, elapsed)

            # Direct file saved log
            print(f"[{saved_count:05d}] Saved -> {selected_sub}/{filename} ({fps:.1f} FPS)")

    except KeyboardInterrupt:
        print("\n\n[INFO] Stopped by user (Ctrl+C).")
    finally:
        ser.close()
        print(f"\n[DONE] Total {saved_count} color images saved to:\n       -> {target_dir}\n")

if __name__ == "__main__":
    main()

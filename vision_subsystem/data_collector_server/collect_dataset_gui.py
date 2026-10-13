"""
NeuroMotion - High-Speed Dataset Collector Client (Python)
Fetches 96x96 frames over Wi-Fi HTTP from XIAO ESP32-S3 and saves directly
to target training folders with customizable naming and interval.

Usage:
    python collect_dataset_gui.py --ip 192.168.1.50 --folder noglasses_open --rate 20
"""

import os
import time
import argparse
import urllib.request
import numpy as np
import cv2

def main():
    parser = argparse.ArgumentParser(description="NeuroMotion Dataset Collector")
    parser.add_argument("--ip", type=str, required=True, help="ESP32 IP address (e.g. 192.168.1.100)")
    parser.add_argument("--outdir", type=str, default=r"C:\Users\Xavier\Documents\Kaggle Projects (DA & DS)\Neuromotion (Drowsiness Detection)\ov2640_samples", help="Base output directory")
    parser.add_argument("--folder", type=str, default="noglasses_open", choices=["noglasses_open", "noglasses_close", "glasses_open", "glasses_close"], help="Subdirectory label")
    parser.add_argument("--prefix", type=str, default="xavier", help="Subject / Session prefix for filename")
    parser.add_argument("--fps", type=float, default=20.0, help="Target capture FPS")
    args = parser.parse_args()

    target_dir = os.path.join(args.outdir, args.folder)
    os.makedirs(target_dir, exist_ok=True)

    url = f"http://{args.ip}/raw"
    delay_sec = 1.0 / args.fps
    
    print("=" * 60)
    print(f" NeuroMotion Dataset Collector")
    print(f" Source URL   : {url}")
    print(f" Target Folder: {target_dir}")
    print(f" Target FPS   : {args.fps} FPS")
    print("=" * 60)
    print("Controls in Window:")
    print("  [SPACE] : Start / Pause Recording")
    print("  [1]     : Set to noglasses_open")
    print("  [2]     : Set to noglasses_close")
    print("  [3]     : Set to glasses_open")
    print("  [4]     : Set to glasses_close")
    print("  [Q]     : Quit")
    print("=" * 60)

    recording = False
    saved_count = 0
    current_folder = args.folder

    cv2.namedWindow("NeuroMotion Collector", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("NeuroMotion Collector", 400, 450)

    while True:
        t0 = time.time()
        try:
            req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
            with urllib.request.urlopen(req, timeout=2.0) as response:
                raw_bytes = response.read()

            if len(raw_bytes) == 96 * 96:
                img = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((96, 96))
                
                # Upscale for clear display
                display = cv2.resize(img, (384, 384), interpolation=cv2.INTER_NEAREST)
                display_bgr = cv2.cvtColor(display, cv2.COLOR_GRAY2BGR)

                # Save frame if recording
                if recording:
                    saved_count += 1
                    filename = f"{current_folder}_{args.prefix}_{saved_count:05d}_{int(time.time()*1000)}.jpg"
                    dest_path = os.path.join(args.outdir, current_folder, filename)
                    cv2.imwrite(dest_path, img)

                # Draw UI text
                status_text = f"REC: {saved_count}" if recording else "PAUSED"
                status_color = (0, 0, 255) if recording else (255, 255, 0)
                
                cv2.putText(display_bgr, f"[{current_folder}]", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_bgr, status_text, (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

                cv2.imshow("NeuroMotion Collector", display_bgr)

        except Exception as e:
            print(f"[Warning] Frame fetch failed: {e}")
            time.sleep(0.5)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break
        elif key == ord(' '):
            recording = not recording
            print(f"[STATUS] Recording {'STARTED' if recording else 'PAUSED'}")
        elif key == ord('1'):
            current_folder = "noglasses_open"
            os.makedirs(os.path.join(args.outdir, current_folder), exist_ok=True)
            print(f"[MODE] Switched to {current_folder}")
        elif key == ord('2'):
            current_folder = "noglasses_close"
            os.makedirs(os.path.join(args.outdir, current_folder), exist_ok=True)
            print(f"[MODE] Switched to {current_folder}")
        elif key == ord('3'):
            current_folder = "glasses_open"
            os.makedirs(os.path.join(args.outdir, current_folder), exist_ok=True)
            print(f"[MODE] Switched to {current_folder}")
        elif key == ord('4'):
            current_folder = "glasses_close"
            os.makedirs(os.path.join(args.outdir, current_folder), exist_ok=True)
            print(f"[MODE] Switched to {current_folder}")

        elapsed = time.time() - t0
        sleep_time = max(0.001, delay_sec - elapsed)
        time.sleep(sleep_time)

    cv2.destroyAllWindows()
    print(f"\nDone. Total captured images: {saved_count}")

if __name__ == "__main__":
    main()

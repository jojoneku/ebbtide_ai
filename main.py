import time
import threading
import cv2

from config import *
from camera_module import init_picamera, get_bgr_frame
from detection_module import load_model, detect_objects, draw_detections
from feedback_module import setup_led, control_feedback, beep_pixhawk
import pixhawk_module  # <-- Correct way to access shared connection state

def main():
    print("[INFO] Starting system...")

    # --- Hardware Initialization ---
    led_line = setup_led()
    cam = init_picamera(DISPLAY_SIZE)

    print("[INFO] Initializing Pixhawk connection...")
    pixhawk_module.init_pixhawk()
    threading.Thread(target=pixhawk_module.connection_monitor, daemon=True).start()

    print("[INFO] Loading YOLO model...")
    model = load_model(MODEL_PATH)
    labels = model.names
    print("Model loaded. Classes:")
    for idx, name in labels.items():
        print(f"  {idx}: {name}")

    # --- State ---
    prev_detected = False
    fps_buffer = []

    while True:
        t_start = time.time()

        # --- Get Frame ---
        frame_raw = get_bgr_frame(cam)

        # --- Run Detection ---
        detections = detect_objects(model, frame_raw)
        trash_detected = any(
            int(d.cls.item()) == 2 and d.conf.item() > THRESHOLD for d in detections
        )

        # --- Draw Detections ---
        frame_out = draw_detections(frame_raw.copy(), detections, labels, SHOW_ALL)

        # --- Feedback ---
        prev_detected = control_feedback(trash_detected, led_line, prev_detected, beep_pixhawk)

        # --- FPS Calculation ---
        fps = 1 / (time.time() - t_start)
        fps_buffer.append(fps)
        if len(fps_buffer) > 100:
            fps_buffer.pop(0)
        avg_fps = sum(fps_buffer) / len(fps_buffer)

        # --- HUD Overlay ---
        cv2.putText(frame_out, f'FPS: {avg_fps:.2f}', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame_out, f'Trash Detected: {"Yes" if trash_detected else "No"}',
            (10, 50), cv2.FONT_HERSHEY_SIMPLEX,0.6, (0, 255, 0) if trash_detected else (0, 0, 255),  # Green if yes, Red if no
2
        )


        # LIVE Pixhawk connection check via correct reference
        pixhawk_status = pixhawk_module.connection_established
        cv2.putText(frame_out, f'Pixhawk: {"Connected" if pixhawk_status else "Disconnected"}', (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if pixhawk_status else (0, 0, 255), 2)

        # --- Display Output ---
        cv2.imshow("Trash Detection", frame_out)

        # --- User Input ---
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('b'):
            beep_pixhawk("notify")

    # --- Cleanup ---
    led_line.set_value(0)
    cam.stop()
    cv2.destroyAllWindows()
    print(f"[INFO] Exiting cleanly. Average FPS: {avg_fps:.2f}")

if __name__ == "__main__":
    main()

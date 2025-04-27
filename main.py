# main.py

import time
import threading
import cv2
from geopy.distance import geodesic

from config import (
    MODEL_PATH, THRESHOLD, DISPLAY_SIZE, SHOW_ALL,
    KNOWN_TRASH_HEIGHT, FOCAL_LENGTH_PX,
    COLLECTION_CONFIRM_RADIUS, COLLECTION_CONFIRM_DELAY
)
from camera_module import init_picamera, get_bgr_frame
from detection_module import load_model, detect_objects, draw_detections
from feedback_module import setup_led, control_feedback, beep_pixhawk
import pixhawk_module
from gps_estimator import (
    infer_fov_from_mode,
    estimate_gps_from_detection,
    estimate_distance_from_bbox
)
from decision_engine import DecisionEngine

# Fake test mode for indoor operation
test_mode = True
FAKE_LAT = 37.4287
FAKE_LON = -122.1719
FAKE_HEADING = 180.0
CURRENT_MISSION_WP = (37.4287, -122.1719)

def main():
    print("[INFO] Starting EbbTide AI system...")

    led_line = setup_led()
    cam = init_picamera(DISPLAY_SIZE)

    pixhawk_module.init_pixhawk()
    threading.Thread(target=pixhawk_module.connection_monitor, daemon=True).start()

    model = load_model(MODEL_PATH)
    labels = ['Human', 'Nontrash', 'Trash']
    fov_degrees = infer_fov_from_mode(DISPLAY_SIZE)

    engine = DecisionEngine()
    prev_detected = False
    fps_buffer = []
    current_target = None
    current_target_time = None

    while True:
        t_start = time.time()
        frame_raw = get_bgr_frame(cam)

        detections = detect_objects(model, frame_raw)
        trash_detected = any(int(d.cls.item()) == 2 and d.conf.item() > THRESHOLD for d in detections)
        frame_out = draw_detections(frame_raw.copy(), detections, labels, SHOW_ALL)
        prev_detected = control_feedback(trash_detected, led_line, prev_detected, beep_pixhawk)

        try:
            if test_mode:
                current_lat = FAKE_LAT
                current_lon = FAKE_LON
                heading_deg = FAKE_HEADING
            else:
                pos_msg = pixhawk_module.mavlink_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
                hud_msg = pixhawk_module.mavlink_connection.recv_match(type='VFR_HUD', blocking=True, timeout=2)
                if pos_msg and hud_msg:
                    current_lat = pos_msg.lat / 1e7
                    current_lon = pos_msg.lon / 1e7
                    heading_deg = hud_msg.heading / 100.0
                else:
                    raise RuntimeError("No GPS/HUD telemetry")

            # --- Confirm trash collection when within radius + delay ---
            if current_target and current_target_time:
                dist_to_target = geodesic((current_lat, current_lon), current_target).meters
                elapsed = time.time() - current_target_time

                if dist_to_target <= COLLECTION_CONFIRM_RADIUS and elapsed > COLLECTION_CONFIRM_DELAY:
                    engine.mark_collected(*current_target)
                    current_target = None
                    current_target_time = None

        except Exception as e:
            print(f"[WARN] GPS read failed: {e}")
            current_lat = current_lon = heading_deg = None

        # --- Process trash detections ---
        if trash_detected and pixhawk_module.connection_established and current_lat is not None:
            try:
                wp_lat, wp_lon = CURRENT_MISSION_WP
                trash_boxes = [d for d in detections if int(d.cls.item()) == 2]

                for det in trash_boxes:
                    xyxy = det.xyxy.cpu().numpy().squeeze()
                    bbox_center_x = (xyxy[0] + xyxy[2]) / 2
                    bbox_height = xyxy[3] - xyxy[1]

                    est_distance_m = estimate_distance_from_bbox(
                        bbox_height_px=bbox_height,
                        frame_height_px=DISPLAY_SIZE[1],
                        known_object_height_m=KNOWN_TRASH_HEIGHT,
                        focal_length_px=FOCAL_LENGTH_PX
                    )

                    est_lat, est_lon = estimate_gps_from_detection(
                        bbox_center_x=bbox_center_x,
                        frame_width=DISPLAY_SIZE[0],
                        current_lat=current_lat,
                        current_lon=current_lon,
                        current_heading_deg=heading_deg,
                        fov_degrees=fov_degrees,
                        est_distance_m=est_distance_m
                    )

                    decision = engine.evaluate_detection(
                        trash_lat=est_lat,
                        trash_lon=est_lon,
                        boat_lat=current_lat,
                        boat_lon=current_lon,
                        wp_lat=wp_lat,
                        wp_lon=wp_lon
                    )

                    if decision["action"] == "collect":
                        print("[ACTION] Approved → Navigating to trash...")
                        current_target = (decision["lat"], decision["lon"])
                        current_target_time = time.time()
                        # TODO: Add command to Pixhawk to go to current_target
                        break

                    elif decision["action"] == "wait":
                        print(f"[WAIT] Trash near WP, but boat not close: {decision['reason']}")
                    else:
                        print(f"[SKIP] Detection ignored: {decision['reason']}")

            except Exception as e:
                print(f"[ERROR] Trash detection failed: {e}")

        # --- FPS display ---
        fps = 1 / (time.time() - t_start)
        fps_buffer.append(fps)
        if len(fps_buffer) > 100:
            fps_buffer.pop(0)
        avg_fps = sum(fps_buffer) / len(fps_buffer)

        # --- UI overlays ---
        cv2.putText(frame_out, f'FPS: {avg_fps:.2f}', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame_out, f'Trash Detected: {"Yes" if trash_detected else "No"}',
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if trash_detected else (0, 0, 255), 2)
        cv2.putText(frame_out, f'Pixhawk: {"Connected" if pixhawk_module.connection_established else "Disconnected"}',
                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if pixhawk_module.connection_established else (0, 0, 255), 2)

        cv2.imshow("Trash Detection", frame_out)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('b'):
            beep_pixhawk("notify")

    led_line.set_value(0)
    cam.stop()
    cv2.destroyAllWindows()
    print(f"[INFO] Shutdown complete. Avg FPS: {avg_fps:.2f}")

if __name__ == "__main__":
    main()

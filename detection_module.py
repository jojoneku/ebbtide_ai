import cv2
from ultralytics import YOLO
from config import INFERENCE_SIZE, THRESHOLD

def load_model(path):
    return YOLO(path, task='detect')

def detect_objects(model, frame):
    resized = cv2.resize(frame, INFERENCE_SIZE)
    results = model(resized, verbose=False)
    return results[0].boxes

def draw_detections(frame, detections, labels, show_all=True):
    h, w = frame.shape[:2]
    for det in detections:
        cls_id = int(det.cls.item())
        conf = det.conf.item()

        if conf < THRESHOLD:
            continue
        if not show_all and cls_id != 2:  # Only show trash if not showing all
            continue

        xyxy = det.xyxy.cpu().numpy().squeeze()
        xmin = int(xyxy[0] * w / INFERENCE_SIZE[0])
        ymin = int(xyxy[1] * h / INFERENCE_SIZE[1])
        xmax = int(xyxy[2] * w / INFERENCE_SIZE[0])
        ymax = int(xyxy[3] * h / INFERENCE_SIZE[1])

        label = f"{labels[cls_id]}: {int(conf * 100)}%"
        color = get_class_color(cls_id)

        cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
        cv2.putText(frame, label, (xmin, ymin - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    return frame

def get_class_color(class_id):
    # Class ID: 0 = human, 1 = nontrash, 2 = trash
    colors = {
        0: (0, 255, 0),
        1: (255, 255, 0),
        2: (0, 0, 255)
    }
    return colors.get(class_id, (255, 255, 255))

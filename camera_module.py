from picamera2 import Picamera2
import cv2

def init_picamera(resolution):
    cam = Picamera2()
    config = cam.create_video_configuration(main={"format": 'XRGB8888', "size": resolution})
    cam.configure(config)
    try:
        cam.set_controls({"AfMode": 1, "Sharpness": 5.0})
    except:
        pass
    cam.start()
    return cam

def get_bgr_frame(cam):
    bgra = cam.capture_array()
    return cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)

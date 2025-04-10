import gpiod
from config import LED_PIN
import pixhawk_module


# --- Setup LED on GPIO ---
def setup_led():
    """
    Initialize and return the LED GPIO line.
    """
    chip = gpiod.Chip('/dev/gpiochip4')
    line = chip.get_line(LED_PIN)
    line.request(consumer="LED", type=gpiod.LINE_REQ_DIR_OUT)
    return line

# --- Control LED + Pixhawk Beep Feedback ---
def control_feedback(trash_detected, led_line, previous_detected, buzzer_func):
    """
    Handles LED lighting and buzzer logic based on trash detection state.
    """
    if trash_detected:
        led_line.set_value(1)
        if not previous_detected:
            buzzer_func("warn")
            print("🚨 Trash detected!")
    else:
        led_line.set_value(0)
    return trash_detected

def beep_pixhawk(pattern="warn"):
    """
    Send a tone command to the Pixhawk via MAVLink.
    """
    if not pixhawk_module.connection_established:
        print("Cannot beep: No Pixhawk connection")
        return False

    tones = {
        "notify": "MFT200L8G",
        "warn": "MFT200L8G>G>G",
        "danger": "MFT200L4>A#A",
        "error": "MFT200L4<B#B"
    }
    tune = tones.get(pattern, "MFT200L8G")

    try:
        print(f"Sending tune command: {tune}")
        try:
            # Newer MAVLink version
            pixhawk_module.mavlink_connection.mav.play_tune_send(
                pixhawk_module.mavlink_connection.target_system,
                pixhawk_module.mavlink_connection.target_component,
                0,
                bytearray([ord(c) for c in tune])
            )
        except:
            # Older fallback
            pixhawk_module.mavlink_connection.mav.play_tune_send(
                pixhawk_module.mavlink_connection.target_system,
                pixhawk_module.mavlink_connection.target_component,
                bytearray([ord(c) for c in tune])
            )
        print(f"Beep command sent: {pattern}")
        return True
    except Exception as e:
        print(f"Failed to send beep command: {e}")
        return False


print("[DEBUG] connection_established in beep_pixhawk:", pixhawk_module.connection_established)
import time
from pymavlink import mavutil
from config import PIXHAWK_CONNECTION, PIXHAWK_BAUD

connection_established = False
mavlink_connection = None

def init_pixhawk():
    global mavlink_connection, connection_established
    try:
        print(f"[DEBUG] Connecting to {PIXHAWK_CONNECTION} @ {PIXHAWK_BAUD}...")
        mavlink_connection = mavutil.mavlink_connection(PIXHAWK_CONNECTION, baud=PIXHAWK_BAUD)
        
        print("[DEBUG] Waiting for heartbeat...")
        heartbeat = mavlink_connection.wait_heartbeat(timeout=10)

        if heartbeat is None:
            print("❌ No heartbeat received from Pixhawk.")
            connection_established = False
            return False

        connection_established = True
        print(f"✅ Connected to Pixhawk: SYS {mavlink_connection.target_system}, COMP {mavlink_connection.target_component}")
        return True

    except Exception as e:
        print(f"❌ Pixhawk connection error: {e}")
        connection_established = False
        return False


def connection_monitor():
    global connection_established, mavlink_connection
    last_heartbeat = time.time()
    while True:
        time.sleep(1)
        try:
            msg = mavlink_connection.recv_match(type='HEARTBEAT', blocking=False)
            if msg:
                last_heartbeat = time.time()
            elif time.time() - last_heartbeat > 5:
                connection_established = False
                mavlink_connection.close()
        except:
            connection_established = False

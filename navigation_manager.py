from pymavlink import mavutil
from geopy.distance import geodesic
from config import NAV_ALTITUDE

def send_guided_waypoint(mavlink, lat, lon, alt=NAV_ALTITUDE):
    print(f"[NAV] Sending GUIDED waypoint to lat={lat:.7f}, lon={lon:.7f}")
    mavlink.mav.set_mode_send(
        mavlink.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4  # GUIDED
    )
    mavlink.mav.mission_item_send(
        mavlink.target_system,
        mavlink.target_component,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2, 0,
        0, 0, 0, 0,
        lat, lon, alt
    )

def jump_to_mission_item(mavlink, seq):
    print(f"[NAV] Jumping to mission item #{seq}")
    mavlink.mav.command_long_send(
        mavlink.target_system,
        mavlink.target_component,
        mavutil.mavlink.MAV_CMD_DO_JUMP,
        0,
        seq, 1,
        0, 0, 0, 0, 0
    )

def find_nearest_wp(current_lat, current_lon, waypoints):
    best_dist = float('inf')
    best_idx = 0
    for i, (lat, lon) in enumerate(waypoints):
        d = geodesic((current_lat, current_lon), (lat, lon)).meters
        if d < best_dist:
            best_dist = d
            best_idx = i
    return best_idx

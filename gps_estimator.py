import math

def infer_fov_from_mode(resolution, sensor_resolution=(4608, 2592), full_fov=102.0):
    """
    Estimate effective horizontal FOV based on active camera resolution and full sensor width.
    """
    active_width = resolution[0]
    sensor_width = sensor_resolution[0]
    crop_ratio = active_width / sensor_width
    estimated_fov = full_fov * crop_ratio
    print(f"[FOV] Resolution: {resolution}, Estimated FOV: {estimated_fov:.2f}°")
    return estimated_fov


def estimate_distance_from_bbox(bbox_height_px, frame_height_px, known_object_height_m=0.3, focal_length_px=1000):
    """
    Estimate distance to object using pinhole camera model.
    """
    if bbox_height_px <= 1:
        return 0  # avoid div by 0
    distance = (known_object_height_m * focal_length_px) / bbox_height_px
    print(f"[DISTANCE ESTIMATE] Bbox height: {bbox_height_px:.1f}px → Estimated distance: {distance:.2f}m")
    return distance


def estimate_gps_from_detection(
    bbox_center_x,
    frame_width,
    current_lat,
    current_lon,
    current_heading_deg,
    fov_degrees,
    est_distance_m
):
    """
    Estimate GPS location of object based on camera detection and bearing.
    """
    pixel_offset = bbox_center_x - (frame_width / 2)
    angle_per_pixel = fov_degrees / frame_width
    relative_angle_deg = pixel_offset * angle_per_pixel
    bearing_deg = (current_heading_deg + relative_angle_deg) % 360
    bearing_rad = math.radians(bearing_deg)

    # Convert lat/lon to radians
    lat1 = math.radians(current_lat)
    lon1 = math.radians(current_lon)
    R = 6378137.0  # Earth radius in meters
    ang_dist = est_distance_m / R

    # Projected GPS point
    lat2 = math.asin(math.sin(lat1) * math.cos(ang_dist) +
                     math.cos(lat1) * math.sin(ang_dist) * math.cos(bearing_rad))
    lon2 = lon1 + math.atan2(math.sin(bearing_rad) * math.sin(ang_dist) * math.cos(lat1),
                             math.cos(ang_dist) - math.sin(lat1) * math.sin(lat2))

    est_lat = math.degrees(lat2)
    est_lon = math.degrees(lon2)

    print(f"[GPS ESTIMATE] lat={est_lat:.7f}, lon={est_lon:.7f}, bearing={bearing_deg:.2f}°, distance={est_distance_m:.2f}m")
    return est_lat, est_lon

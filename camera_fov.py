def infer_fov_from_mode(resolution, sensor_resolution=(4608, 2592), full_fov=102.0):
    """
    Estimates the effective horizontal FOV based on camera resolution crop.

    Args:
        resolution (tuple): (width, height) of the active video mode (e.g., (1536, 864))
        sensor_resolution (tuple): Full sensor size, defaults to IMX708: 4608x2592
        full_fov (float): Full horizontal FOV of the Pi Camera v3 Wide (in degrees)

    Returns:
        float: Estimated horizontal FOV for current mode
    """
    active_width = resolution[0]
    sensor_width = sensor_resolution[0]
    crop_ratio = active_width / sensor_width

    estimated_fov = full_fov * crop_ratio
    print(f"[FOV] Resolution: {resolution}, Estimated FOV: {estimated_fov:.2f}°")
    return estimated_fov

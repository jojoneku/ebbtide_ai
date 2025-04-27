from pymavlink import mavutil

# Establish connection
connection = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)

# Wait for a heartbeat before sending commands
connection.wait_heartbeat()
print(f"Heartbeat received from system (system {connection.target_system} component {connection.target_component})")

# Mission Message
# Complete mission definition for a drone

std_msgs/Header header

string mission_id
string mission_name  
string mission_type          # survey, search, delivery, etc.

Waypoint[] waypoints         # List of waypoints

# Mission parameters
float32 default_speed        # m/s
float32 default_altitude     # meters
bool return_to_home          # RTH after mission complete

# Safety parameters
float32 max_altitude         # Maximum altitude limit
float32 min_battery_return   # Battery % to trigger RTH
geometry_msgs/Polygon geofence # Boundary polygon (optional)

# Metadata
string created_by
builtin_interfaces/Time created_at

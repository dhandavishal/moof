# Drone Status Message
# Comprehensive status information for a single drone

# Header
std_msgs/Header header

# Drone Identification
uint8 drone_id
string drone_name

# Connection Status
bool connected
bool armed
string mode

# Position Information (NED frame)
geometry_msgs/Point position          # x, y, z in meters
geometry_msgs/Vector3 velocity        # vx, vy, vz in m/s
float32 altitude_agl                  # Altitude above ground level
float32 altitude_msl                  # Altitude above mean sea level

# Orientation
geometry_msgs/Quaternion orientation
float32 heading                       # Degrees, 0-360

# Battery Information
float32 battery_voltage               # Volts
float32 battery_percentage            # 0-100%
float32 battery_current               # Amperes
int32 battery_remaining_time          # Seconds

# GPS Information
bool gps_fix
uint8 gps_satellites
float32 gps_hdop

# Mission Status
string current_task                   # Current task being executed
uint8 task_progress                   # 0-100%
string mission_state                  # IDLE, EXECUTING, PAUSED, etc.

# Health Status
bool system_healthy
string[] warnings
string[] errors

# Timestamp of last update
builtin_interfaces/Time last_update

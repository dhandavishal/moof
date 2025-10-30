# Command message sent from TEE to FAL
# Instructs FAL to execute a specific flight primitive

string command_id           # Unique identifier for this command
string primitive_type       # goto, loiter, land, takeoff, rtl

# Target parameters (used by goto, loiter, takeoff)
geometry_msgs/Point target_position
geometry_msgs/Quaternion target_orientation

# Velocity parameters
float32 velocity            # m/s (travel speed)
float32 climb_rate          # m/s (for takeoff/land)

# Position tolerance
float32 acceptance_radius   # meters (waypoint acceptance)
float32 yaw_tolerance       # radians (heading tolerance)

# Loiter-specific parameters
float32 loiter_radius       # meters (orbit radius)
float32 loiter_duration     # seconds (how long to loiter)

# Execution parameters
float32 timeout             # seconds (max execution time)
bool blocking               # Wait for completion before next primitive

# Coordinate frame
string frame_id             # Coordinate frame (map, body, etc.)

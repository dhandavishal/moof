# Waypoint Message
# Represents a single waypoint in a mission

uint32 sequence              # Waypoint number in mission
geometry_msgs/Point position # x, y, z in meters (NED frame)
float32 heading              # Desired heading at waypoint (-1 = don't care)
float32 speed                # Desired speed to waypoint (0 = default)
float32 acceptance_radius    # How close to get to waypoint (meters)
uint8 action                 # Action at waypoint (0=none, 1=hover, 2=take_photo, etc.)
float32 hold_time            # Time to hold at waypoint (seconds)

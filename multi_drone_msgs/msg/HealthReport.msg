# Health report message for component monitoring
# Published by health monitors to report system status

string component         # Component name (battery, gps, overall, etc.)
string state             # Health state: good, warning, critical, unknown
float32 value            # Numeric value (percentage, count, etc.)
string message           # Human-readable status message
builtin_interfaces/Time timestamp  # Report timestamp

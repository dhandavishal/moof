# Status feedback from FAL to TEE
# Reports execution progress of a primitive command

string command_id           # Matches PrimitiveCommand.command_id
string status               # executing, completed, failed, timeout, cancelled
float32 progress            # 0.0 to 1.0 (execution progress)
string error_message        # If failed, description of error
builtin_interfaces/Time timestamp  # Status update time

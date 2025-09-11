# moof
ROS2 and aersostack2 based "Mission Oriented Operating Framework" for Software Defined Drones


#DISCLAIMER for Third-Party Repositories  
_All external code is used **only** for academic exploration and non-commercial research.  
Each project remains under its original license and copyright._

| Folder in this workspace | Original Repository | Brief purpose in my research |
|--------------------------|---------------------|------------------------------|
| `aerostack2/` | https://github.com/aerostack2/aerostack2 | Full-stack ROS 2 framework for multi-drone missions; studied for autonomous swarm control. |
| `as2_platform_mavlink/` | https://github.com/aerostack2/as2_platform_mavlink | MAVLink interface layer that lets Aerostack 2 communicate with ArduPilot/PX4; required for hardware-in-the-loop tests. |
| `project_as2_multirotor_simulator/` | https://github.com/aerostack2/project_as2_multirotor_simulator | Gazebo/Ignition simulator configuration for testing swarm behaviors before real-world flights. |
| `project_gazebo/` | https://github.com/aerostack2/project_gazebo | Supplementary Gazebo world files and robot models leveraged for environmental realism in simulation. |

## Attribution

Please refer to each upstream repository for its license and contributing guidelines. If you use any of these components beyond personal research, comply with the respective project’s license requirements.

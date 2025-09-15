Relevant source files

-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/crazyflie/crazyflie.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/crazyflie/crazyflie.sdf.jinja)
-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/gimbal/model.config](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/gimbal/model.config)
-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/gimbal/position\_gimbal.sdf](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/gimbal/position_gimbal.sdf)
-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/gimbal/position\_gimbal.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/gimbal/position_gimbal.sdf.jinja)
-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/gimbal/speed\_gimbal.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/gimbal/speed_gimbal.sdf.jinja)
-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/hd\_camera/hd\_camera.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/hd_camera/hd_camera.sdf.jinja)
-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/hexrotor\_base/hexrotor\_base.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/hexrotor_base/hexrotor_base.sdf.jinja)
-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/px4vision/px4vision.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/px4vision/px4vision.sdf.jinja)
-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/quadrotor\_base/quadrotor\_base.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/quadrotor_base/quadrotor_base.sdf.jinja)
-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/rgbd\_camera/rgbd\_camera.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/rgbd_camera/rgbd_camera.sdf.jinja)
-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/semantic\_camera/semantic\_camera.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/semantic_camera/semantic_camera.sdf.jinja)
-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/vga\_camera/vga\_camera.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/vga_camera/vga_camera.sdf.jinja)
-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/x500/x500.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/x500/x500.sdf.jinja)

## Purpose and Overview

This page documents the simulation capabilities within Aerostack2 (AS2). The simulation infrastructure provides a physically accurate virtual environment for developing and testing drone applications without the risks associated with real hardware. Based on Gazebo/Ignition Gazebo, the simulation system includes realistic models for various drones, sensors, physics, and control systems.

For information about drone and sensor models specifically, see [Drone and Sensor Models](https://deepwiki.com/aerostack2/aerostack2/4.1-drone-and-sensor-models).

## Simulation Architecture

The simulation infrastructure in Aerostack2 is organized around a set of components that work together to provide a comprehensive simulation environment.

```
Aerostack2 Coreas2_platform_gazeboGazebo SimulatorDrone ModelsSensor ModelsPhysics EngineControl Pluginsquadrotor_base.sdf.jinjacrazyflie.sdf.jinjax500.sdf.jinjapx4vision.sdf.jinjahexrotor_base.sdf.jinjahd_camera.sdf.jinjavga_camera.sdf.jinjargbd_camera.sdf.jinjasemantic_camera.sdf.jinjaposition_gimbal.sdf.jinjaspeed_gimbal.sdf.jinjaMulticopterMotorModelMulticopterVelocityControlMulticopterINDIControlJointPositionControllerJointController
```

Sources: [as2\_simulation\_assets/as2\_gazebo\_assets/models/](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/)

## Drone Model Structure

Each drone model in the simulation is built from multiple components that work together to create a realistic virtual representation of the aircraft.

```
Drone Model (namespace)Base Drone ModelPose Publisher PluginMotor Model PluginsController PluginOdometry PluginBattery PluginSensor ModelsIMU PluginRotor 0 - MulticopterMotorModelRotor 1 - MulticopterMotorModelRotor 2 - MulticopterMotorModelRotor 3 - MulticopterMotorModelMulticopterVelocityControlMulticopterINDIControlCamera SensorsGimbal SystemsHD CameraVGA CameraRGBD CameraSemantic CameraPosition GimbalSpeed Gimbal
```

Sources: [as2\_simulation\_assets/as2\_gazebo\_assets/models/quadrotor\_base/quadrotor\_base.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/quadrotor_base/quadrotor_base.sdf.jinja) [as2\_simulation\_assets/as2\_gazebo\_assets/models/crazyflie/crazyflie.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/crazyflie/crazyflie.sdf.jinja) [as2\_simulation\_assets/as2\_gazebo\_assets/models/x500/x500.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/x500/x500.sdf.jinja)

## Supported Drone Models

Aerostack2 provides several drone models for simulation, each implemented as an SDF (Simulation Description Format) file with Jinja2 templating for customization.

### Quadrotor Base

The quadrotor\_base is a generic quadcopter model that serves as the foundation for other drone models. It includes:

-   Four rotors with configurable properties
-   IMU sensor
-   Optional velocity controller
-   Optional acrobatic (INDI) controller
-   Optional battery model
-   Support for attaching additional sensors

Source: [as2\_simulation\_assets/as2\_gazebo\_assets/models/quadrotor\_base/quadrotor\_base.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/quadrotor_base/quadrotor_base.sdf.jinja)

### Crazyflie

The Crazyflie model simulates the small, lightweight Crazyflie drone. It follows the same plugin structure as quadrotor\_base but with parameters tailored to the Crazyflie's characteristics.

Source: [as2\_simulation\_assets/as2\_gazebo\_assets/models/crazyflie/crazyflie.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/crazyflie/crazyflie.sdf.jinja)

### X500

The X500 model represents a medium-sized quadcopter suitable for carrying various sensors and payloads. It includes:

-   Four rotors with higher maximum rotation velocity (1000.0 vs 800.0)
-   Different controller gains optimized for the X500's dynamics
-   Compatible with all sensor models

Source: [as2\_simulation\_assets/as2\_gazebo\_assets/models/x500/x500.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/x500/x500.sdf.jinja)

### PX4Vision

The PX4Vision model is compatible with the PX4 autopilot ecosystem and features different motor and moment constants, as well as controller parameters tuned for its specific aerodynamics.

Source: [as2\_simulation\_assets/as2\_gazebo\_assets/models/px4vision/px4vision.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/px4vision/px4vision.sdf.jinja)

### Hexrotor Base

The hexrotor\_base model is a six-rotor drone that provides enhanced stability and payload capacity. Unlike the quadcopter models, it has six motor model plugins and a different rotor configuration in the controller.

Source: [as2\_simulation\_assets/as2\_gazebo\_assets/models/hexrotor\_base/hexrotor\_base.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/hexrotor_base/hexrotor_base.sdf.jinja)

## Sensor Models

### Camera Sensors

Aerostack2 supports four types of camera sensors:

| Camera Type | Description | Resolution | Features |
| --- | --- | --- | --- |
| HD Camera | High-definition RGB camera | 1280x960 | Customizable lens parameters |
| VGA Camera | Standard resolution camera | 1280x960 | Customizable lens parameters |
| RGBD Camera | Combined RGB and depth camera | 640x480 | Configurable depth range |
| Semantic Camera | For semantic image segmentation | 1280x960 | Instance or semantic segmentation modes |

Source: [as2\_simulation\_assets/as2\_gazebo\_assets/models/hd\_camera/hd\_camera.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/hd_camera/hd_camera.sdf.jinja) [as2\_simulation\_assets/as2\_gazebo\_assets/models/semantic\_camera/semantic\_camera.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/semantic_camera/semantic_camera.sdf.jinja)

### Gimbal Models

Gimbals allow cameras to be movable, providing pan, tilt, and roll capabilities. Aerostack2 includes two types of gimbals:

```
sensor.name_joint (fixed)sensor.name_yaw_joint (revolute)sensor.name_pitch_joint (revolute)sensor.name_roll_joint (revolute)sensor_attached_joint (fixed)Topic: /namespace/sensor.name/gimbal_cmd/position/0Topic: /namespace/sensor.name/gimbal_cmd/position/1Topic: /namespace/sensor.name/gimbal_cmd/position/2Gimbal Model (sensor.name)base_linkdrone base_linkyaw_pitch_linkpitch_roll_linksensor_attached_enu_linkSensor ModelController PluginsRoll Joint ControllerPitch Joint ControllerYaw Joint Controller
```

Sources: [as2\_simulation\_assets/as2\_gazebo\_assets/models/gimbal/position\_gimbal.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/gimbal/position_gimbal.sdf.jinja) [as2\_simulation\_assets/as2\_gazebo\_assets/models/gimbal/speed\_gimbal.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/gimbal/speed_gimbal.sdf.jinja)

#### Position Gimbal

The position gimbal allows precise positioning of a camera or other sensor using position control. It includes:

-   Three revolute joints (roll, pitch, yaw)
-   Position controllers with PID parameters for each axis
-   Topics for commanding each joint position
-   Joint state publisher for feedback

Source: [as2\_simulation\_assets/as2\_gazebo\_assets/models/gimbal/position\_gimbal.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/gimbal/position_gimbal.sdf.jinja)

#### Speed Gimbal

The speed gimbal controls camera movement by velocity commands, suitable for smooth camera motion. It includes:

-   Three revolute joints (roll, pitch, yaw)
-   Velocity controllers for each axis
-   Topics for commanding each joint velocity
-   Joint state publisher for feedback

Source: [as2\_simulation\_assets/as2\_gazebo\_assets/models/gimbal/speed\_gimbal.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/gimbal/speed_gimbal.sdf.jinja)

## Physics and Control Plugins

### Motor Model Plugin

The `ignition-gazebo-multicopter-motor-model-system` plugin simulates realistic motor dynamics with parameters for thrust, torque, and aerodynamic effects:

```
<plugin
    filename="ignition-gazebo-multicopter-motor-model-system"
    name="ignition::gazebo::systems::MulticopterMotorModel">
    <robotNamespace>model/{{ namespace }}</robotNamespace>
    <jointName>rotor_0_joint</jointName>
    <linkName>rotor_0</linkName>
    <turningDirection>ccw</turningDirection>
    <timeConstantUp>0.0125</timeConstantUp>
    <timeConstantDown>0.025</timeConstantDown>
    <maxRotVelocity>800.0</maxRotVelocity>
    <motorConstant>8.54858e-06</motorConstant>
    <momentConstant>0.016</momentConstant>
    <commandSubTopic>command/motor_speed</commandSubTopic>
    <motorNumber>0</motorNumber>
    <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
    <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
    <motorSpeedPubTopic>motor_speed/0</motorSpeedPubTopic>
    <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
    <motorType>velocity</motorType>
</plugin>
```

Key parameters include:

-   `timeConstantUp`/`timeConstantDown`: Motor response time constants
-   `maxRotVelocity`: Maximum motor rotation speed
-   `motorConstant`: Constant relating motor speed to thrust
-   `momentConstant`: Constant relating motor speed to torque
-   `rotorDragCoefficient`: Coefficient for aerodynamic drag
-   `rollingMomentCoefficient`: Coefficient for rolling moment

Source: [as2\_simulation\_assets/as2\_gazebo\_assets/models/quadrotor\_base/quadrotor\_base.sdf.jinja23-43](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/quadrotor_base/quadrotor_base.sdf.jinja#L23-L43)

### Velocity Controller Plugin

The `gz-sim-multicopter-control-system` plugin provides a velocity controller with:

-   Linear velocity control
-   Attitude control
-   Angular rate control
-   Configurable gains and limits
-   Optional noise simulation

Key parameters include:

-   `velocityGain`: Gains for linear velocity control
-   `attitudeGain`: Gains for attitude control
-   `angularRateGain`: Gains for angular rate control
-   `maximumLinearAcceleration`/`maximumLinearVelocity`: Motion limits
-   Noise parameters for realistic behavior

Source: [as2\_simulation\_assets/as2\_gazebo\_assets/models/quadrotor\_base/quadrotor\_base.sdf.jinja106-124](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/quadrotor_base/quadrotor_base.sdf.jinja#L106-L124)

### INDI Controller Plugin

The `libMulticopterINDIControl.so` plugin implements an Incremental Nonlinear Dynamic Inversion controller for acrobatic flight with parameters for:

-   PID gains for attitude control
-   Incremental control parameters (alpha)
-   Anti-windup and saturation configurations

Source: [as2\_simulation\_assets/as2\_gazebo\_assets/models/quadrotor\_base/quadrotor\_base.sdf.jinja127-143](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/quadrotor_base/quadrotor_base.sdf.jinja#L127-L143)

### Gimbal Controller Plugins

#### Position Controller

The `ignition-gazebo-joint-position-controller-system` plugin provides position control for gimbal joints with PID parameters:

```
<plugin
    filename="ignition-gazebo-joint-position-controller-system"
    name="gz::sim::systems::JointPositionController">
    <joint_name>{{ sensor.name }}_roll_joint</joint_name>
    <topic>/{{ namespace }}/{{ sensor.name }}/gimbal_cmd/position/0</topic>
    <p_gain>1.0</p_gain>
    <i_gain>0.0</i_gain>
    <d_gain>0.5</d_gain>
    <cmd_max>1</cmd_max>
    <cmd_min>1</cmd_min>
    <use_velocity_commands>true</use_velocity_commands>
</plugin>
```

Source: [as2\_simulation\_assets/as2\_gazebo\_assets/models/gimbal/position\_gimbal.sdf.jinja147-158](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/gimbal/position_gimbal.sdf.jinja#L147-L158)

#### Speed Controller

The `ignition-gazebo-joint-controller-system` plugin provides velocity control for gimbal joints:

```
<plugin
    filename="ignition-gazebo-joint-controller-system"
    name="ignition::gazebo::systems::JointController">
    <joint_name>{{ sensor.name }}_roll_joint</joint_name>
    <topic>/{{ namespace }}/{{ sensor.name }}/gimbal_cmd/twist/0</topic>
</plugin>
```

Source: [as2\_simulation\_assets/as2\_gazebo\_assets/models/gimbal/speed\_gimbal.sdf.jinja147-151](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/gimbal/speed_gimbal.sdf.jinja#L147-L151)

## Additional Simulation Plugins

### Odometry Plugin

The `ignition-gazebo-odometry-publisher-system` plugin publishes odometry information at a configurable frequency:

```
<plugin
    filename="ignition-gazebo-odometry-publisher-system"
    name="ignition::gazebo::systems::OdometryPublisher">
    <dimensions>3</dimensions>
    <odom_publish_frequency>100</odom_publish_frequency>
</plugin>
```

Source: [as2\_simulation\_assets/as2\_gazebo\_assets/models/quadrotor\_base/quadrotor\_base.sdf.jinja176-183](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/quadrotor_base/quadrotor_base.sdf.jinja#L176-L183)

### Battery Plugin

The `libignition-gazebo-linearbatteryplugin-system.so` plugin simulates battery discharge with parameters for:

-   Initial charge and capacity
-   Voltage characteristics
-   Discharge behavior
-   Power load

Source: [as2\_simulation\_assets/as2\_gazebo\_assets/models/quadrotor\_base/quadrotor\_base.sdf.jinja185-200](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/quadrotor_base/quadrotor_base.sdf.jinja#L185-L200)

### IMU Plugin

The `ignition-gazebo-imu-system` plugin simulates an inertial measurement unit:

```
<plugin
    filename="ignition-gazebo-imu-system"
    name="ignition::gazebo::systems::Imu">
</plugin>
```

Source: [as2\_simulation\_assets/as2\_gazebo\_assets/models/quadrotor\_base/quadrotor\_base.sdf.jinja248-251](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/quadrotor_base/quadrotor_base.sdf.jinja#L248-L251)

## Integration with Aerostack2

The simulation infrastructure integrates with the rest of Aerostack2 through the platform abstraction layer, which provides a common interface for both simulated and real drones.

```
Aerostack2 CorePlatform Abstractionas2_platform_gazeboGazebo SimulatorROS 2 TopicsMotion CommandsSensor DataState InformationBehavior SystemPython API
```

This integration allows:

1.  The same behaviors to be used in both simulation and real-world scenarios
2.  Seamless transition from simulation to real hardware
3.  Testing of complex missions before deployment on physical drones

Sources: [as2\_simulation\_assets/as2\_gazebo\_assets/models/quadrotor\_base/quadrotor\_base.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/quadrotor_base/quadrotor_base.sdf.jinja) [as2\_simulation\_assets/as2\_gazebo\_assets/models/crazyflie/crazyflie.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/crazyflie/crazyflie.sdf.jinja) [as2\_simulation\_assets/as2\_gazebo\_assets/models/x500/x500.sdf.jinja](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/x500/x500.sdf.jinja)
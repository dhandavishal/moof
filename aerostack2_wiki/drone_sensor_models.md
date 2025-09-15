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

## Introduction

This document provides a comprehensive overview of the drone and sensor models available in the Aerostack2 simulation environment. These models form the foundation of simulation capabilities in Aerostack2, allowing users to simulate various drone platforms and sensor configurations using the Gazebo/Ignition simulation engine.

The models defined here are used by the Gazebo Platform component (see [section 4.2](https://deepwiki.com/aerostack2/aerostack2/4.2-gazebo-integration)) to create realistic simulations of UAVs with various sensors. This document focuses on the physical models and their configurations, rather than on how to set up and run simulations (covered in [section 4.2](https://deepwiki.com/aerostack2/aerostack2/4.2-gazebo-integration)) or the platform abstraction layer (covered in [section 6.2](https://deepwiki.com/aerostack2/aerostack2/6.2-gazebo-platform)).

Sources:

-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/quadrotor\_base/quadrotor\_base.sdf.jinja1-254](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/quadrotor_base/quadrotor_base.sdf.jinja#L1-L254)
-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/crazyfline/crazyfline.sdf.jinja1-254](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/crazyfline/crazyfline.sdf.jinja#L1-L254)

## Architecture Overview

The drone and sensor models in Aerostack2 follow a modular architecture that allows for flexible configuration and composition:

```
ControllersSensorsDrone ModelsextendsextendsextendsextendsQuadrotor BaseX500CrazyfliePX4 VisionHexrotor BaseCameras- HD Camera- VGA Camera- RGBD Camera- Semantic CameraGimbals- Position Gimbal- Speed GimbalIMU SensorOdometry PluginBattery PluginVelocity ControllerAcrobatic ControllerPosition Controller
```

Sources:

-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/quadrotor\_base/quadrotor\_base.sdf.jinja1-254](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/quadrotor_base/quadrotor_base.sdf.jinja#L1-L254)
-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/crazyflie/crazyflie.sdf.jinja1-254](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/crazyflie/crazyflie.sdf.jinja#L1-L254)
-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/x500/x500.sdf.jinja1-243](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/x500/x500.sdf.jinja#L1-L243)

## Drone Models

Aerostack2 provides several drone models with different physical characteristics and capabilities:

| Model | Description | Key Parameters |
| --- | --- | --- |
| Quadrotor Base | Generic quadrotor model | 4 rotors, base model for other quadrotors |
| X500 | Medium-sized quadrotor platform | Max rotor velocity: 1000 rad/s |
| Crazyflie | Micro quadrotor | Lightweight, smaller form factor |
| PX4 Vision | PX4-based quadrotor | Based on PX Vision airframe |
| Hexrotor | Six-rotor platform | Higher payload capacity, 6 rotors |

All drone models include:

-   Base physical model with appropriate inertial properties
-   Motor models with realistic dynamics
-   IMU sensor system
-   Optional odometry plugin
-   Optional battery model
-   Support for various controllers (velocity, acrobatic)

### Common Drone Model Components

Each drone model in Aerostack2 follows a common structure:

```
hasusesincorporates1114-611..*DroneModel+base_link+rotors[]+sensors[]+controllers[]+plugins[]MotorModel+turningDirection+maxRotVelocity+motorConstant+momentConstant+timeConstantsController+VelocityController+INDIControllerPlugin+IMUSystem+OdometryPublisher+BatteryPlugin+PosePublisher
```

Sources:

-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/quadrotor\_base/quadrotor\_base.sdf.jinja22-103](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/quadrotor_base/quadrotor_base.sdf.jinja#L22-L103)
-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/x500/x500.sdf.jinja23-90](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/x500/x500.sdf.jinja#L23-L90)

### Motor Configuration

Drone models include motor configurations with the following parameters:

| Parameter | Description | Example Value |
| --- | --- | --- |
| turningDirection | Rotation direction (CW/CCW) | "ccw" or "cw" |
| timeConstantUp | Rise time constant | 0.0125 |
| timeConstantDown | Fall time constant | 0.025 |
| maxRotVelocity | Maximum rotor velocity | 800.0-1000.0 rad/s |
| motorConstant | Force constant | 8.54858e-06 |
| momentConstant | Moment constant | 0.016 |
| rotorDragCoefficient | Drag coefficient | 8.06428e-05 |
| rotorVelocitySlowdownSim | Slowdown factor | 10 |

Each drone model specifies rotor configurations based on its physical design, with alternating rotation directions to counterbalance torque.

Sources:

-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/quadrotor\_base/quadrotor\_base.sdf.jinja22-102](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/quadrotor_base/quadrotor_base.sdf.jinja#L22-L102)
-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/crazyflie/crazyflie.sdf.jinja22-101](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/crazyflie/crazyflie.sdf.jinja#L22-L101)

## Sensor Models

Aerostack2 provides several sensor models that can be attached to drones:

### Camera Models

Four camera types are available:

1.  **HD Camera** - High-definition camera:
    
    -   Resolution: 1280×960
    -   Field of view: 1.0472 rad
    -   Horizontal focal length: 1108.5 pixels
    -   Vertical focal length: 1108.5 pixels
    -   Update rate: 20 Hz
2.  **VGA Camera** - Standard resolution camera:
    
    -   Resolution: 640×480
    -   Similar parameters to HD camera but lower resolution
3.  **RGBD Camera** - Combined RGB and depth camera:
    
    -   RGB resolution: 640×480
    -   Depth sensing range: 0.1m to 10m
    -   Update rate: 20 Hz
4.  **Semantic Camera** - Camera for semantic segmentation:
    
    -   Provides RGB image and segmentation data
    -   Can be configured for instance or semantic segmentation
    -   Update rate: 30 Hz

Sources:

-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/hd\_camera/hd\_camera.sdf.jinja15-71](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/hd_camera/hd_camera.sdf.jinja#L15-L71)
-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/semantic\_camera/semantic\_camera.sdf.jinja14-106](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/semantic_camera/semantic_camera.sdf.jinja#L14-L106)
-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/rgbd\_camera/rgbd\_camera.sdf.jinja14-76](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/rgbd_camera/rgbd_camera.sdf.jinja#L14-L76)
-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/vga\_camera/vga\_camera.sdf.jinja15-72](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/vga_camera/vga_camera.sdf.jinja#L15-L72)

### Gimbal Models

Aerostack2 provides two types of gimbals for mounting sensors:

1.  **Position Gimbal** - Controlled by setting absolute angular positions:
    
    -   3-axis control (roll, pitch, yaw)
    -   Position-based control interface
    -   Joint state publisher for feedback
    -   PID controllers for each axis
2.  **Speed Gimbal** - Controlled by setting angular velocities:
    
    -   3-axis control (roll, pitch, yaw)
    -   Velocity-based control interface
    -   Joint state publisher for feedback

Both gimbal types allow mounting any of the camera models, creating a stabilized sensor platform.

```
Control InterfacesGimbal Structureyaw_jointpitch_jointroll_jointfixed_joint/gimbal_cmd/position/[0-2]/gimbal_cmd/position/[0-2]/gimbal_cmd/position/[0-2]/gimbal_cmd/twist/[0-2]/gimbal_cmd/twist/[0-2]/gimbal_cmd/twist/[0-2]base_linkyaw_pitch_linkpitch_roll_linksensor_enu_linkCamera/SensorPosition Controller(position_gimbal)Speed Controller(speed_gimbal)
```

Sources:

-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/gimbal/position\_gimbal.sdf.jinja1-186](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/gimbal/position_gimbal.sdf.jinja#L1-L186)
-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/gimbal/speed\_gimbal.sdf.jinja1-168](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/gimbal/speed_gimbal.sdf.jinja#L1-L168)

## Additional Sensors and Plugins

### IMU Sensor

Every drone model includes an IMU (Inertial Measurement Unit) by default. The IMU provides:

-   Angular velocity measurements
-   Linear acceleration measurements
-   Orientation data

The IMU is implemented using Gazebo's built-in IMU plugin:

```
<plugin
    filename="ignition-gazebo-imu-system"
    name="ignition::gazebo::systems::Imu">
</plugin>
```

Source:

-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/quadrotor\_base/quadrotor\_base.sdf.jinja248-251](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/quadrotor_base/quadrotor_base.sdf.jinja#L248-L251)

### Odometry Plugin

An optional odometry publisher can be enabled to provide odometry information:

```
<plugin
    filename="ignition-gazebo-odometry-publisher-system"
    name="ignition::gazebo::systems::OdometryPublisher">
    <dimensions>3</dimensions>
    <odom_publish_frequency>100</odom_publish_frequency>
</plugin>
```

Source:

-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/quadrotor\_base/quadrotor\_base.sdf.jinja175-180](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/quadrotor_base/quadrotor_base.sdf.jinja#L175-L180)

### Battery Plugin

An optional battery model can be enabled to simulate battery discharge:

```
<plugin filename="libignition-gazebo-linearbatteryplugin-system.so"
    name="ignition::gazebo::systems::LinearBatteryPlugin">
    <battery_name>linear_battery</battery_name>
    <voltage>12.694</voltage>
    <open_circuit_voltage_constant_coef>12.694</open_circuit_voltage_constant_coef>
    <open_circuit_voltage_linear_coef>0</open_circuit_voltage_linear_coef>
    <initial_charge>{{ capacity }}</initial_charge>
    <capacity>{{ capacity }}</capacity>
    <resistance>0.061523</resistance>
    <smooth_current_tau>1.9499</smooth_current_tau>
    <power_load>6.6</power_load>
    <start_on_motion>true</start_on_motion>
</plugin>
```

Source:

-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/quadrotor\_base/quadrotor\_base.sdf.jinja185-198](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/quadrotor_base/quadrotor_base.sdf.jinja#L185-L198)

## Control Plugins

Each drone model can be equipped with different control plugins:

### Velocity Controller

The velocity controller allows direct control of the drone's velocity:

```
<plugin
  filename="gz-sim-multicopter-control-system"
  name="gz::sim::systems::MulticopterVelocityControl">
  <robotNamespace>model/{{ namespace }}</robotNamespace>
  <commandSubTopic>cmd_vel</commandSubTopic>
  <enableSubTopic>velocity_controller/enable</enableSubTopic>
  <comLinkName>base_link</comLinkName>
  <velocityGain>3.3 3.3 3.5</velocityGain>
  <attitudeGain>5.0 5.0 0.9</attitudeGain>
  <angularRateGain>1.5 1.5 0.09</angularRateGain>
  <maximumLinearAcceleration>2 2 2</maximumLinearAcceleration>
  <maximumLinearVelocity>5 5 5</maximumLinearVelocity>
  <maximumAngularVelocity>3 3 3</maximumAngularVelocity>
</plugin>
```

Source:

-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/x500/x500.sdf.jinja93-121](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/x500/x500.sdf.jinja#L93-L121)

### INDI Acrobatic Controller

An incremental nonlinear dynamic inversion (INDI) controller for acrobatic maneuvers:

```
<plugin 
  filename="libMulticopterINDIControl.so"
  name="gz::sim::systems::MulticopterINDIControl">
  <robotNamespace>model/{{ namespace }}</robotNamespace>
  <commandSubTopic>acro</commandSubTopic>
  <enableSubTopic>model/{{ namespace }}/acro_controller/enable</enableSubTopic>
  <comLinkName>base_link</comLinkName>
  <Kp_gains>12.0 12.0 8.0</Kp_gains>
  <Ki_gains>0.0 0.0 0.0</Ki_gains>
  <Kd_gains>2.15 2.15 0.0</Kd_gains>
  <alpha>0.6 0.6 0.6</alpha>
  <antiwindup_cte>1.0 1.0 1.0</antiwindup_cte>
  <reset_integral_flag>1</reset_integral_flag>
  <proportional_saturation_flag>1</proportional_saturation_flag>
</plugin>
```

Source:

-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/x500/x500.sdf.jinja115-142](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/x500/x500.sdf.jinja#L115-L142)

## Model Composition System

Aerostack2 uses a template-based approach to generate complete robot models by combining drone platforms with various sensors:

```
Sensor CompositionModel Definitionfor each sensorconnectsconnectsincludesSDF Template (.sdf.jinja)Configuration ParametersTemplate RendererFinal SDF ModelBase Drone ModelSensor Specifications- Type (camera, gimbal, etc.)- Pose (position/orientation)- ParametersInclude Sensor ModelCreate Joint
```

This system allows for flexible configuration of drones with different sensor payloads by:

1.  Selecting a base drone model (quadrotor, hexrotor, etc.)
2.  Adding sensors with specified positions and orientations
3.  Configuring controller parameters
4.  Enabling optional plugins (odometry, battery, etc.)

Sources:

-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/quadrotor\_base/quadrotor\_base.sdf.jinja202-247](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/quadrotor_base/quadrotor_base.sdf.jinja#L202-L247)
-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/x500/x500.sdf.jinja190-235](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/x500/x500.sdf.jinja#L190-L235)

## Sensor Integration

Sensors are attached to drone models using a consistent pattern:

```
{% for sensor in sensors -%}
    <!-- Payload {{ sensor.model }} -->
    {% if sensor.model == 'gimbal_position' -%}
        {% include 'gimbal/position_gimbal.sdf.jinja' with context %}
    {% elif sensor.model == 'gimbal_speed' -%}
        {% include 'gimbal/speed_gimbal.sdf.jinja' with context %}
    {% elif sensor.model == 'hd_camera' and not sensor.gimbaled -%}
        {% include 'hd_camera/hd_camera.sdf.jinja' with context %}
    {% elif sensor.model == 'vga_camera' and not sensor.gimbaled -%}
        {% include 'vga_camera/vga_camera.sdf.jinja' with context %}
    {% elif sensor.model == 'semantic_camera' and not sensor.gimbaled -%}
        {% include 'semantic_camera/semantic_camera.sdf.jinja' with context %}
    {% elif sensor.model == 'rgbd_camera' and not sensor.gimbaled -%}
        {% include 'rgbd_camera/rgbd_camera.sdf.jinja' with context %}
    {% elif sensor.gimbaled -%}
        {# Special handling for gimbaled sensors #}
    {% else -%}
        <include>
            <name>{{ sensor.name }}</name>
            <uri>model://{{ sensor.model }}</uri>
            <pose relative_to="base_link">{{ sensor.pose }}</pose>
        </include>
        <joint name="{{ sensor.name }}_joint" type="fixed">
            <parent>base_link</parent>
            <child>{{ sensor.name }}</child>
        </joint>
    {% endif -%}
{% endfor -%}
```

This templating approach allows for:

1.  Attaching various sensor types directly to the drone body
2.  Mounting sensors on gimbals for stabilized sensing
3.  Configuring sensor positions and orientations

Sources:

-   [as2\_simulation\_assets/as2\_gazebo\_assets/models/quadrotor\_base/quadrotor\_base.sdf.jinja202-247](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_simulation_assets/as2_gazebo_assets/models/quadrotor_base/quadrotor_base.sdf.jinja#L202-L247)

## Conclusion

The drone and sensor models in Aerostack2 provide a flexible framework for simulating various UAV platforms with different sensor configurations. The modular design allows for customization through configuration parameters, enabling users to create realistic simulations tailored to their specific requirements.

These models serve as the foundation for the Gazebo simulation integration (see [section 4.2](https://deepwiki.com/aerostack2/aerostack2/4.2-gazebo-integration)), which handles the initialization and management of the simulation environment.
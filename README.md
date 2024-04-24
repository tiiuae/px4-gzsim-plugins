

# Gazebo Sim Plugins
## MAVLink plugin for HITL
A Bridge between Gzsim and Px4 in HITL mode via Mavlink

### Mavlink subscribed messages (from PX4)
| Type                            | Description                        |
|---------------------------------|------------------------------------|
| HIL_ACTUATOR_CONTROLS           |  Actuator control outputs from PX4 |

### Mavlink published messages (to PX4)
| Type                 | Description                                                                                            |
|----------------------|--------------------------------------------------------------------------------------------------------|
| HIL_SENSOR           | The IMU readings in SI units in NED body frame, Magnetic field, absolute and differential pressure|
| HIL_STATE_QUATERNION | Used mainly to send gz model pose info|
| HIL_GPS              | The global position, as returned by the Global Positioning System (GPS). This is NOT the global position estimate of the system, but rather a RAW sensor value. Send now mainly to give home GPS coordinate to PX4|


### Gz subscribed topics (from Gzsim)
| Topic name                             | Type             | Description                                |
|----------------------------------------|------------------|--------------------------------------------|
| /link/base_link/sensor/imu_sensor/imu  | gz::msgs::IMU    | IMU sensor data                            |
| /pose/info                             | gz::msgs::Pose_V | Position and orientation vector of a model |


### Gzsim plugins affected
| Plugin name                             | Description                                                              |
|-----------------------------------------|--------------------------------------------------------------------------|
| gz::sim::systems::MulticopterMotorModel | This system applies a thrust force to models with spinning propellers    |



## ABC plugin for HITL/SITL


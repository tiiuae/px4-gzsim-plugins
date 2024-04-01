<img align="right" height="20" src="https://auterion.com/wp-content/uploads/2020/05/auterion_logo_default_sunrise.svg">

# Ignition Gazebo for MAVLink SITL and HITL

[![Build Tests](https://github.com/Auterion/sitl_ign_gazebo/actions/workflows/build_test.yml/badge.svg)](https://github.com/Auterion/sitl_ign_gazebo/actions/workflows/build_test.yml)

This is a Software-In-The-Loop/Hardware-In-The-Loop simulation environment for the PX4 autopilot project with [Ignition Gazebo](https://ignitionrobotics.org/home)

## Installation

Follow instructions on the [official site](http://gazebosim.org/tutorials?cat=install) to install Gazebo.

```
sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
apt update
apt install ignition-edifice
```

## Running the simulation
The simulation can be run using the following command at the root of the [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) repository
```
make px4_sitl ignition
```

For more instructions when running the simulation with PX4, follow the [documentation](http://docs.px4.io/master/en/simulation/ignition_gazebo.html)

## Creation and install of debian package on Linux
This creates a debian package and installs the plugins to /usr/lib on Linux. To use this package you must add /usr/lib to IGN_GAZEBO_SYSTEM_PLUGIN_PATH.
```
mkdir build
cd build
cmake ..
make
cpack -G DEB
sudo dpkg -i *.deb
```

### Tips
For the [**_collision plugin_**](./src/harmonic_collision.cpp) a model must have
1. Collision
```
<collision name='collision'>
    <geometry>
        <box>
            <size>2.0 1.0 0.5</size>
        </box>
    </geometry>
</collision>
```
 2. Contact sensor
 ```
<sensor name='sensor_contact' type='contact'>
    <contact>
        <!--The name of the collision element from above-->
        <collision>collision</collision>
    </contact>
</sensor>
 ```
3. Plugins 
```
<plugin filename="gz-sim-contact-system"
        name="gz::sim::systems::Contact">
</plugin>
<plugin filename="collision_plugin"
        name="gz::CollisionPlugin">
    <!-- 1000 == 1 sec -->
    <updateRate>1000</updateRate>
</plugin>
```

For the [**_groundtruth plugin_**](./src/groundtruth_plugin.cpp) a model must have only plugin
```
<plugin filename="groundtruth_plugin"
    name="gz::sim::systems::GroundtruthPlugin">
    <!-- 1000 == 1 sec -->
    <update_frequency>100</update_frequency>
</plugin>
```

Collision and groundtruth plugins requires [TASFMessage](https://github.com/tiiuae/TASFMessages) library for work
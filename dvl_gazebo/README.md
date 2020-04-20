# dvl_gazebo ROS Package
Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.  You may obtain a copy of the License at

https://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the License for the specific language governing permissions and limitations under the License.

## Overview
The ROS `dvl_gazebo` package provides SDF, URDF, and Xacro files for utilization, testing, and evaluation of the UUV Simulator Doppler Velocity ogger (DVL) plugin.  The included models and plugin parameters roughly align with the specifications of the Teledyne Workhorse Navigator 600.  Model and world files can be used as is for testing of the DVL model.  The Xacro file can be used to add a DVL to an existing robot model.

The package contains two launch files that can be run independently (both covered in more detail below) that can be executed with the following commands:

```
roslaunch dvl_gazebo teledyne_whn_standalone.launch
```
OR
```
roslaunch dvl_gazebo test_teledyne_whn.launch
```

Once Gazebo is up, choosing to "Follow" the `teledyne_whn` model simplifies the visualization.

In addition, the package contains URDF and SDF models and Xacro macros for use in implementing DVLs in other Gazebo simulations.

## teledyne\_whn\_standalone.launch

### Launch and Operation
The `teledyne_whn_standalone.launch` file instantiates a Gazebo world (by default, the uuv\_gazebo\_worlds ocean\_waves.world provided by the UUV simulator containing a single Teledyne DVL attached to a dimensionless link (the DVL is not within the world's camera field of view, but the it is located just below the water's surface near the model's center as depicted below).

![Standalone WHN600 DVL at simulation startup](figs/standalone_whn_startup.png  "Standalone WHN600 DVL at simulation startup")

After approximately 10 seconds, the DVL will begin moving in a descending left-hand turn.  It will continue with this pattern until impacting the bottom.  During its descent, the DVL sonar beam visualization will indicate its perceived height above the bottom.

![Standalone WHN600 DVL during simulation run](figs/standalone_whn_running.png  "Standalone WHN600 DVL during simulation run")

The following launch arguments can be used to modify the execution:

- **gui** (default `true`): set to false to run the simulation without the GUI or rendered scene.
- **paused** (default `false`): set to true to start the simulation with physics paused (can be unpaused and repaused with the `/gazebo/pause_physics` and `/gazebo/unpause_physics` services respecively.
- **world_name** (default `uuv_gazebo_worlds/worlds/ocean_waves.world`): used to change the world model that is used when the simulation commences.



### ROS Topics
The following ROS topics are relevant:

- **/dvl/dvl/state** (`std_msgs/Bool`): should be true as long as the DVL is active (can be activated and deactivated with the `/dvl/dvl/change_state` service).
- **/dvl/dvl** (`uuv_sensor_ros_plugins_msgs/DVL`): provides the current DVL-derived linear velocities (relative to the `dvl_base_link`, which is oriented forward-right-down) and covariance, perceived altitude above the bottom (computed as the average of the 4 individual sonar ranges), and the ranges and other information for the 4 individual DVL sonar beams.
- **/dvl/dvl\_twist** (`geometry_msgs/TwistWithCovarianceStamped`): provides the current DVL-derived linear velocities
- **/dvl/dvl\_sonar0**, **/dvl/dvl\_sonar0**, **/dvl/dvl\_sonar0**, and **/dvl/dvl\_sonar0** (`sensor_msgs/Range`): Range and other information for each individual DVL sonar.

### ROS Nodes
#### spawn\_whn (`gazebo_ros/spawn_model)`
The `spawn_whn` node is used to spawn the standalone WHN600 model in the gazebo world.  Once the model has been spawned, this node will terminate.

#### apply\_velocity (`dvl_gazebo/simple_motion.py`)
The `apply_velocity` node publishes periodic `gazebo_msgs/ModelState` messages to the `/gazebo/set_model_state` topic to control the motion of the DVL model (fixed speed forward at 1 meter per second, down at 0.5 meters per second, and left turn at 0.25 radians per second).  The model name and frame to which the new state is relative are provided as ROS parameters (set in the launch file).  The frame of relative motion should be the model's base link to ensure correct motion.

#### joint\_state\_publisher (`joint_state_publisher/joint_state_publisher`)
The `joint_state_publisher` node continually publishes the state of the model's joints to the `/joint_states` topic as it moves through the world so that they will be available to the robot state publisher.

#### robot\_state\_publisher (`robot_state_publisher/robot_state_publisher`)
The `robot_state_publisher` node subscribes to the `/joint_states` topic and publishes transforms to the `/tf` topic.  The DVL plugin uses the transforms associated with the 4 DVL sonar beams in its calculations.

See launch file for additional information regarding parameters and arguments.

## test\_teledyne\_whn.launch

### Launch and Operation
The `test_teledyne_whn.launch` file uses macros from a Xacro file to generate a WHN600 DVL model mounted on a simple medium-sized "UUV".  The cylindrical UUV is 4 meters in length, 1 meter in diameter, and has a mass or 200 kilograms.  As with the standalone DVL example, the model is not within the world's camera field of view, but the it is located just below the water's surface near the model's center as depicted below.

![Xacro-generated WHN600 DVL mounted to a simple UUV at startup](figs/mounted_whn_startup.png  "Xacro-generated WHN600 DVL mounted to a simple UUV at startup")

After approximately 10 seconds, the UUVwill begin moving in a descending left-hand turn.  It will continue with this pattern until impacting the bottom.  During its descent, the DVL sonar beam visualization will indicate its perceived height above the bottom.

![Xacro-generated WHN600 DVL mounted to a simple UUV during simulation run](figs/mounted_whn_running.png  "Xacro-generated WHN600 DVL mounted to a simple UUV during simulation run")

Launch arguments, ROS nodes, and relevant ROS topics are the same as with the standalone WHN600 example.

## Models and Macros
### Teledyne WHN URDF (`models/teledyne_whn_urdf/model.urdf`)
The `teledyne_whn_urdf` model is a self-contained implementation of the Teledyne WND600 DVL in URDF format.  This model can be directly loaded into the ROS parameter server and added to Gazebo scenes with a `gazebo_ros/spawn_model` node.  The standalone WHN600 model is connected to a dimensionless link so that it points directly down.  If this model is to be used as the basis for incorporation into a specific URDF robot model, it can be added to the model by copying everything after the `dvl_base_link` definition into the robot's URDF file and using the `dvl_base_joint` to connect the DVL to the robot's base link.

In order for the DVL plugin to work correctly, it must have access to the ROS transforms for the robot link to which it is attached.  These may be published by the plugin controlling the robot body or by a combination of joint and robot state publishers (as in the standalone DVL example).

### Teledyne WHN (`models/teledyne_whn/model.sdf`)
The `teledyne whn` model is a self-contained implementation of the Teledyne WND600 DVL in SDF format.  This model can be included in any SDF world and connected to a robot in that world through a joint connecting the model's `dvl_link` to the robot's base link.

In order for the DVL plugin to work correclty, it must have access to the ROS transforms for the robot link to which it is attached.  Because SDF is not compatible with the ROS robot state publisher, the transforms must be published from a different source (most likely from the robot's controller plugin).

**NOTE**:  The DVL plugin for this model has not been tested.

### Teledyne WHN Macros (`urdf/teledyne_whn.xacro`)
The `teledyne_whn` Xacro file provides a set of macros for the addition of a Teledyne WHN600 DVL to an arbitrary Xacro-generated robot model (see `urdf/test_teledyne_whn.xacro` for a simple example).  Four top-level macros are provided:

- **dvl\_macro**: provides for the generation of a DVL model with a user-specified namespace, parent link (robot base link), inertial reference frame, and origin relative to the inertial reference frame.
- **dvl\_sensor\_enu**: provides for the generation of a DVL model in an east-north-up inertial reference frame (i.e., the default Gazebo frame) with a user-specified namespace, parent link, and origin relative to the inertial reference frame.  The `test_teledyne_whn.launch` example utilizes this macro.
- **dvl\_sensor\_ned**: provides for the generation of a DVL model in a north-east-down inertial reference frame with a user-specified namespace, parent link, and origin relative to the inertial reference frame.  The NED frame must be explicitly defined or included in the world model (see `worlds/uuv_dave_ocean_waves_watch_dvl.world`).
- **dvl\_plugin\_macro**: provides for the generation of a DVL model with user-specified namespace, suffix (sensor ID), parent link, individual sonar ROS topic names, visual scale, update rate, sensor noise parameters, inertial reference frame, and origin relative to the inertial reference frame.  This macro provides the most flexibility and can be used to model most real-world DVLs.  The `dvl_macro` macro is essentially a wrapper for this macro and can be used as an example for its use.

**NOTE for all models and macros**: Gravity has been disabled for all of these models for the time being so that they can be incorporated without additional plugins and controllers.
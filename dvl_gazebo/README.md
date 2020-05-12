# dvl_gazebo ROS Package
Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.  You may obtain a copy of the License at

https://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the License for the specific language governing permissions and limitations under the License.

## Overview
The ROS `dvl_gazebo` package provides SDF, URDF, and Xacro files for utilization, testing, and evaluation of the UUV Simulator  and Woods Hole Oceanographic Institute (WHOI) Deep Submergence Laboratory (DSL) environment ROS Doppler Velocity Logger (DVL) plugins.  The included exemplar models and plugin parameters utilized by the fully-functional launch files described in this tutorial roughly align with the specifications of the Teledyne Workhorse Navigator 600.  Model and world files can be used as is for testing of the DVL model.  The Xacro files can be used to add a DVL to an existing robot model.

Xacro files are provided for the following DVLs are provided in addition to the WHN 600 of the executable examples:

- **Nortek DVL 500-300m**
- **Nortek DVL 500-6000m**
- **Nortek DVL 1000-300m**
- **Nortek DVL 1000-4000m**
- **Sonardyne Syrinx 600**
- **Teledyne Explorer 1000 (Phased Array)**
- **Teledyne Explorer 4000 (Piston)**

These templates can be utilized in a manner similar to those associated with the Teledyne WHN 600.  For the time being, these files utilize the WHN 600 visual model but implement correct collision, mass, and performance characteristics (per the respective vendor-provided data sheets).  Updated visual models will be added to the package as the meshes are developed.

## UUV Simulator Plugin Utilization

The package contains two launch files that can be run independently (both covered in more detail below) to launch DVL models utilizing the UUV Simulator DVL plugin.  These can be executed with the following commands:

```
roslaunch dvl_gazebo uuvsim_teledyne_whn_standalone.launch
```
OR
```
roslaunch dvl_gazebo uuvsim_teledyne_whn.launch
```

Once Gazebo is up, choosing to "Follow" the `teledyne_whn` model simplifies the visualization.

In addition, the package contains URDF and SDF models and Xacro macros for use in implementing DVLs in other Gazebo simulations.

### uuvsim\_teledyne\_whn\_standalone.launch

#### Launch and Operation
The `uuvsim_teledyne_whn_standalone.launch` file instantiates a Gazebo world (by default, the `uuv_gazebo_worlds/ocean_waves.world` provided by the UUV simulator) containing a single Teledyne DVL attached to a dimensionless link (the DVL is not within the world's camera field of view, but the it is located just below the water's surface near the model's center as depicted below).

![Standalone WHN600 DVL at simulation startup](https://raw.githubusercontent.com/wiki/Field-Robotics-Lab/dave/tutorials/images/standalone_whn_startup.png  "Standalone WHN600 DVL at simulation startup")

After approximately 10 seconds, the DVL will begin moving in a descending left-hand turn.  It will continue with this pattern until impacting the bottom.  During its descent, the DVL sonar beam visualization will indicate its perceived height above the bottom.

![Standalone WHN600 DVL during simulation run](https://raw.githubusercontent.com/wiki/Field-Robotics-Lab/dave/tutorials/images/standalone_whn_running.png  "Standalone WHN600 DVL during simulation run")

The following launch arguments can be used to modify the execution:

- **gui** (default `true`): set to false to run the simulation without the GUI or rendered scene.
- **paused** (default `false`): set to true to start the simulation with physics paused (can be unpaused and repaused with the `/gazebo/pause_physics` and `/gazebo/unpause_physics` services respecively.
- **world_name** (default `uuv_gazebo_worlds/worlds/ocean_waves.world`): used to change the world model that is used when the simulation commences.

#### ROS Topics
The following ROS topics are relevant:

- **/dvl/dvl/state** (`std_msgs/Bool`): should be true as long as the DVL is active (can be activated and deactivated with the `/dvl/dvl/change_state` service).
- **/dvl/dvl** (`uuv_sensor_ros_plugins_msgs/DVL`): provides the current DVL-derived linear velocities (relative to the `dvl_base_link`, which is oriented forward-left-up) and covariance, perceived altitude above the bottom (computed as the average of the 4 individual sonar ranges), and the ranges and other information for the 4 individual DVL sonar beams.
- **/dvl/dvl\_twist** (`geometry_msgs/TwistWithCovarianceStamped`): provides the current DVL-derived linear velocities
- **/dvl/dvl\_sonar0**, **/dvl/dvl\_sonar0**, **/dvl/dvl\_sonar0**, and **/dvl/dvl\_sonar0** (`sensor_msgs/Range`): Range and other information for each individual DVL sonar.

#### ROS Nodes
##### spawn\_whn (`gazebo_ros/spawn_model)`
The `spawn_whn` node is used to spawn the standalone WHN600 model in the gazebo world.  Once the model has been spawned, this node will terminate.

##### apply\_velocity (`dvl_gazebo/simple_motion.py`)
The `apply_velocity` node publishes periodic `gazebo_msgs/ModelState` messages to the `/gazebo/set_model_state` topic to control the motion of the DVL model (fixed speed forward at 1 meter per second, down at 0.5 meters per second, and left turn at 0.25 radians per second).  The model name and frame to which the new state is relative are provided as ROS parameters (set in the launch file).  The frame of relative motion should be the model's base link to ensure correct motion.

##### joint\_state\_publisher (`joint_state_publisher/joint_state_publisher`)
The `joint_state_publisher` node continually publishes the state of the model's joints to the `/joint_states` topic as it moves through the world so that they will be available to the robot state publisher.

##### robot\_state\_publisher (`robot_state_publisher/robot_state_publisher`)
The `robot_state_publisher` node subscribes to the `/joint_states` topic and publishes transforms to the `/tf` topic.  The DVL plugin uses the transforms associated with the 4 DVL sonar beams in its calculations.

See launch file for additional information regarding parameters and arguments.

### uuvsim\_teledyne\_whn.launch

#### Launch and Operation
The `uuvsim_teledyne_whn.launch` file uses macros from a Xacro file to generate a WHN600 DVL model mounted on a simple medium-sized "UUV".  The cylindrical UUV is 4 meters in length, 1 meter in diameter, and has a mass or 200 kilograms.  As with the standalone DVL example, the model is not within the world's camera field of view, but the it is located just below the water's surface near the model's center as depicted below.

![Xacro-generated WHN600 DVL mounted to a simple UUV at startup](https://raw.githubusercontent.com/wiki/Field-Robotics-Lab/dave/tutorials/images/mounted_whn_startup.png  "Xacro-generated WHN600 DVL mounted to a simple UUV at startup")

After approximately 10 seconds, the UUVwill begin moving in a descending left-hand turn.  It will continue with this pattern until impacting the bottom.  During its descent, the DVL sonar beam visualization will indicate its perceived height above the bottom.

![Xacro-generated WHN600 DVL mounted to a simple UUV during simulation run](https://raw.githubusercontent.com/wiki/Field-Robotics-Lab/dave/tutorials/images/mounted_whn_running.png  "Xacro-generated WHN600 DVL mounted to a simple UUV during simulation run")

Launch arguments, ROS nodes, and relevant ROS topics are the same as with the standalone WHN600 example.

### UUV Simulator Models and Macros
#### Teledyne WHN URDF (`models/teledyne_whn_urdf/model.urdf`)
The `teledyne_whn_urdf` model is a self-contained implementation of the Teledyne WND600 DVL incorporating the UUV Simulator plugin in URDF format.  This model can be directly loaded into the ROS parameter server and added to Gazebo scenes with a `gazebo_ros/spawn_model` node.  The standalone WHN600 model is connected to a dimensionless link so that it points directly down.  If this model is to be used as the basis for incorporation into a specific URDF robot model, it can be added to the model by copying everything after the `dvl_base_link` definition into the robot's URDF file and using the `dvl_base_joint` to connect the DVL to the robot's base link.

In order for the DVL plugin to work correctly, it must have access to the ROS transforms for the robot link to which it is attached.  These may be published by the plugin controlling the robot body or by a combination of joint and robot state publishers (as in the standalone DVL example).

#### Teledyne WHN (`models/teledyne_whn/model.sdf`)
The `teledyne whn` model is a self-contained implementation of the Teledyne WND600 DVL in SDF format.  This model can be included in any SDF world and connected to a robot in that world through a joint connecting the model's `dvl_link` to the robot's base link.

In order for the DVL plugin to work correclty, it must have access to the ROS transforms for the robot link to which it is attached.  Because SDF is not compatible with the ROS robot state publisher, the transforms must be published from a different source (most likely from the robot's controller plugin).

**NOTE**:  The DVL plugin for this model has not been tested.

#### Teledyne WHN Macros (`urdf/teledyne_whn.xacro`)
The `urdf/teledyne_whn` Xacro file provides a set of macros for the addition of a Teledyne WHN600 DVL to an arbitrary Xacro-generated robot model (see `urdf/uuvsim_teledyne_whn.xacro` for a simple example).  Four top-level macros are provided:

- **dvl\_macro**: provides for the generation of a DVL model with a user-specified namespace, parent link (robot base link), inertial reference frame, and origin relative to the inertial reference frame.  Parameters are as follows:
-- *namespace*:  string namespace in which all of the links and ROS topics reside.
-- *parent_link*:  robot link to which the sensor link will be attached.
-- *inertial_reference_frame*:  static reference frame (i.e., world frame) in which the robot maneuvers.
-- *\*origin*:  block parameter for the location of the DVL sensor link on the robot.
- **dvl\_sensor\_enu**: provides for the generation of a DVL model in an east-north-up inertial reference frame (i.e., the default Gazebo frame) by invoking the `dvl_macro` macro with appropriate parameters (the `uuvsim_teledyne_whn.launch` example utilizes this macro).  Parameters are as follows:
-- *namespace*:  string namespace in which all of the links and ROS topics reside.
-- *parent_link*:  robot link to which the sensor link will be attached.
-- *\*origin*:  block parameter for the location of the DVL sensor link on the robot.
- **dvl\_sensor\_ned**: provides for the generation of a DVL model in an north-east-down inertial reference frame (i.e., the default Gazebo frame) by invoking the `dvl_macro` macro with appropriate parameters.  The NED frame must be explicitly defined or included in the world model (see `worlds/uuv_dave_ocean_waves_watch_dvl.world`) for models generated with this macro to function properly.  Parameters are as follows:
-- *namespace*:  string namespace in which all of the ROS topics reside.
-- *parent_link*:  robot link to which the sensor link will be attached.
-- *\*origin*:  block parameter for the location of the DVL sensor link on the robot.
- **dvl\_plugin\_macro**: provides for the generation of a DVL model with user-specified namespace, suffix (sensor ID), parent link, individual sonar ROS topic names, visual scale, update rate, sensor noise parameters, inertial reference frame, and origin relative to the inertial reference frame.  This macro provides the most flexibility and can be used to model most real-world DVLs.  The `dvl_macro` macro is essentially a wrapper for this macro and can be used as an example for its use.  Parameters are as follows:
-- *namespace*:  string namespace in which all of the links and ROS topics reside.
-- *suffix*:  arbitrary identifying suffix that is added to all joint and link names.
-- *parent_link*:  robot link to which the sensor link will be attached.
-- *topic*:  ROS topic to which DVL sensor messages are to be published.
-- *update_rate*:  rate at which DVL sensor messages are to be published (Hz).
-- *reference_frame:  static reference frame (i.e., world frame) in which the robot maneuvers.
-- *noise_sigma*:  standard deviation of the velocity solution (only used for covariance matrix computation).
-- *noise_amplitude*:  standard deviation of the Gaussian noise added to each of the computed linear velocity vectors.
-- *\*origin*:  block parameter for the location of the DVL sensor link on the robot.

#### Additional Macro Files

The following Xacro files are provided for the generation of URDF UUV Simulator plugin models for different commercially available DVLs.  Each provide the same top-level macros as `urdf/teledyne_whn.xacro` and implement sensor characteristics as specified in the vendor-provided datasheets.  They can be utilized in the same manner as the macros in `urdf/teledyne_whn.xacro`.

- **Nortek DVL500-300m** (`urdf/nortek_dvl500_300.xacro`)
- **Nortek DVL500-6000m** (`urdf/nortek_dvl500_6000.xacro`)
- **Nortek DVL1000-300m** (`urdf/nortek_dvl1000_300.xacro`)
- **Nortek DVL1000-4000m** (`urdf/nortek_dvl1000_4000.xacro`)
- **Sonardyne Syrinx 600** (`urdf/sonardyne_syrinx_600.xacro`)
- **Teledyne Explorer 1000 (Phased Array)** (`urdf/teledyne_explorer_1000.xacro`)
- **Teledyne Explorer 4000 (Piston)** (`urdf/teledyne_explorer_4000.xacro`)

**NOTE for all models and macros**: Gravity has been disabled for all of these models for the time being so that they can be incorporated without additional plugins and controllers.

## Woods Hole Oceanographic Institute Deep Submergence Lab Environment Plugin Utilization

The WHOI Deep Submergence Lab (DSL) environment plugins require the `ds_sim` and `ds_msgs` ROS packages, both of which are available at https://bitbucket.org/whoidsl/.  Prior to compiling the packages for ROS Melodic (Gazebo 8 and higher), the `ds_sim` repository must be switched to the `feature/melodic` branch.

The `dvl_gazebo` package contains two launch files that can be run independently to launch DVL models utilizing the WHOI DVL plugin.  These launch files function similarly to those launching the UUV Simulator plugin models and can be executed with the following commands:

```
roslaunch dvl_gazebo whoi_teledyne_whn_standalone.launch
```
OR
```
roslaunch dvl_gazebo whoi_teledyne_whn.launch
```

As with their UUV Simulator counterparts, choosing to "Follow" `teledyne_whn` model wil simplify the visualization.

In addition, the package contains an SDF model and Xacro macros for use in implementing DVLs in other Gazebo simulations.

### whoi\_teledyne\_whn\_standalone.launch

#### Launch and Operation
The `whoi_teledyne_whn_standalone.launch` file instantiates a Gazebo world (by default, the `dvl_gazebo/uuv_dave_ocean_waves_watch_whoi_dvl.world`) containing a single Teledyne DVL attached to a dimensionless link (the DVL is not within the world's camera field of view, but the it is located just below the water's surface near the model's center).

After approximately 10 seconds, the model will begin a descending left hand circle as with UUV Simulator plugin standalone demonstration.  The simulation will procede as depicted in the figures above except that the individual sonar beams will not be visible.

Available launch arguments are the same as described for previous launch files.

#### ROS Topics
The following ROS topics are relevant:

- **/dvl/dvl** (`ds_sensor_msgs/Dvl`): Provides current DVL sensor status and information.  Information that is correctly represented are linear velocity and covariance, individual sonar ranges and covariance, individual sonar beam unit vectors (in the DVL reference frame).  Additional message fields are either not currently implemented by the sensor or plugin (e.g., raw velocity and covariance) or are fixed to align with static sensor and plugin implementations (e.g., velocity_mode, coordinate_mode, and dvl_type).
- **/dvl/ranges** (`ds_sensor_msgs/Ranges3D`): provides current range information for each of the DVL's sonar beams.  Information includes range quality and validity metrics, and a 3D point in the DVL reference frame corresponding to its return range (out to maximum range).
- **/dvl/dvl_cloud** (`sensor_msgs/PointCloud`): Point cloud representation of the individual DVL beams.  The published point cloud does not provide any representation of the current ranges or velocities, but can be used to ensure proper pose relative to a robot body (e.g., using RVIZ).

#### ROS Nodes

Unlike the UUV Simulator plugin, the WHOI model does not rely on externally generated transforms to function properly.  Rather, it relies on a custom Gazebo sensor.  This implementation obviates the requirement for `joint_state_publisher` or `robot_state_publisher` nodes.  Further, this implementation allows the DVL model to be included in the world file as SDF rather than spawned dynamically using URDF, so the `spawn_whn` node is also not required.  An `apply_velocity` node is used to move the model through the world.

See launch file for additional information regarding parameters and arguments (including the Gazebo parameters for inclusion of the `ds_sim` DVL sensor).

### whoi\_teledyne\_whn.launch

#### Launch and Operation
The `whoi_teledyne_whn.launch` file uses macros from a Xacro file to generate a WHN600 DVL model mounted on a simple medium-sized "UUV".  The cylindrical UUV is 4 meters in length, 1 meter in diameter, and has a mass or 200 kilograms.  As with the standalone DVL example, the model is not within the world's camera field of view, but the it is located just below the water's surface near the model's center.

After approximately 10 seconds, the UUVwill begin moving in a descending left-hand turn.  It will continue with this pattern until impacting the bottom.  Individual DVL sonar beams are not visible.

Launch arguments and relevant ROS topics are the same as with the standalone WHN600 example.

#### ROS Nodes

Launching ROS with the `whoi_teledyne_whn.launch` file invokes all of the ROS nodes described in the UUV Simulator `uuvsim_teledyne_whn-standalone.launch` section of this tutorial.  The `joint_state_publisher` and `robot_state_publisher` nodes have been specifically included to facilitate RViz utilization.  In addition to those nodes, an `rviz` node is started with a configuration loaded to visualize the point cloud generated by the DVL plugin (see below).

![RViz visualization of WHOI DSL DVL plugin configuration](https://raw.githubusercontent.com/wiki/Field-Robotics-Lab/dave/tutorials/images/whoi_dvl_rviz.png  "RViz visualization of WHOI DSL DVL plugin configuration")

The RViz visualization does not provide any sensed information, but the displayed point cloud can be used to verify the correct placement and orientation of the DVL on the robot.  See `rviz/whoi_teledyne_dvl.rviz` file for specific configuration.
 
### WHOI DSL Models and Macros
#### Teledyne WHN (`models/teledyne_whn_whoi/model.sdf`)
The `teledyne whn_whoi` model is a self-contained implementation of the Teledyne WND600 DVL using the WHOI plugin in SDF format.  This model can be included in any SDF world and connected to a robot in that world through a joint connecting the model's `dvl_base_link` to the robot's base link.  Because the WHOI DSL environment implements a custom Gazebo DVL sensor rather than relying on embedded Ray sensors, this model's plugin does not require access to externally generated transforms and will work as is (this model is loaded into the world file for the `whoi_teledyne_whn_standalone.launch` demo).

#### Teledyne WHN Macros (`urdf_whoi/teledyne_whn.xacro`)
The `urdf_whoi/teledyne_whn` Xacro file provides a set of macros for the addition of a Teledyne WHN600 DVL to an arbitrary Xacro-generated robot model (see `urdf_whoi/whoi_teledyne_whn.xacro` for a simple example).  A single top-level macros is provided:

- **teledyne\_whn\_macro**: provides for the generation of a DVL model with a user-specified parameters.  The macro generates the DVL link, its sensor (both the Gazebo sensor element and plugin), and the joint attaching the DVL to a robot.  Parameters are as follows:
-- *name*:  string name of the sensor that will be incorporated into link, joint, and sensor names.
-- *namespace*:  string namespace in which all of the ROS topics reside.
-- *xyz*:  location on the DVL link at which the visual and collision elements are centered (inertial is centered on the origin).
-- *dvl_topic*:  ROS topic to which DVL sensor messages will be published.
-- *ranges_topic*:  ROS topic to which consolidated sonar beam range messages will be published.
-- *robot_link*:  robot link to which the sensor link will be attached.
-- *joint_xyz*:  position on the robot to which the DVL is to be mounted.

Additional macros provided in `urdf_whoi/teledyne_whn.xacro` (*teledyne\_whn\_link*, *teledyne\_whn\_sensor*, and *teledyne\_whn\_joint*) can be used individually for a more customized link, sensor, and joint configuration if desired, but they have not been tested extensively in this manner.

#### Additional Macro Files

The following Xacro files are provided for the generation of URDF WHOI plugin models for different commercially available DVLs.  Each provide the same top-level macros as `urdf_whoi/teledyne_whn.xacro` and implement sensor characteristics as specified in the vendor-provided datasheets.  They can be utilized in the same manner as the macros in `urdf_whoi/teledyne_whn.xacro`.

- **Nortek DVL500-300m** (`urdf_whoi/nortek_dvl500_300.xacro`)
- **Nortek DVL500-6000m** (`urdf_whoi/nortek_dvl500_6000.xacro`)
- **Nortek DVL1000-300m** (`urdf_whoi/nortek_dvl1000_300.xacro`)
- **Nortek DVL1000-4000m** (`urdf_whoi/nortek_dvl1000_4000.xacro`)
- **Sonardyne Syrinx 600** (`urdf_whoi/sonardyne_syrinx_600.xacro`)
- **Teledyne Explorer 1000 (Phased Array)** (`urdf_whoi/teledyne_explorer_1000.xacro`)
- **Teledyne Explorer 4000 (Piston)** (`urdf_whoi/teledyne_explorer_4000.xacro`)

**NOTE for all models and macros**: Gravity has been disabled for all of these models for the time being so that they can be incorporated without additional plugins and controllers.

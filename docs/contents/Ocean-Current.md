---
layout: default
title: Ocean Currents
nav_exclude: true
---

# Contents

<!-- TOC generated with https://github.com/ekalinin/github-markdown-toc -->
<!--
 cat fls_model_standalone.md | ./gh-md-toc -
-->

* [Background](#background)
  * [Basic definitions and implementation](#basic-definitions-and-implementation)
* [Constant ocean current quickstart](#Constant-ocean-current-quickstart)
* [Stratified ocean current quickstart](#Stratified-ocean-current-quickstart)
* [Multiple vehicle support](#Stratified-ocean-current-plugin-with-multiple-vehicle-support)
* [Tidal Oscillation support](#Tidal-oscillation-support)

This document describes the capabilities of the ocean current plugin and provides tutorials showing how to modify its behavior.

* The ground plane for the stratified ocean current plugin is inherited from the UUV simulator's ocean current plugin. [link](https://github.com/uuvsimulator/uuv_simulator/wiki/Applying-disturbances-during-the-simulation#current-velocity)
* The user must specify mean velocity and direction in the world SDF file.
* The plugin is integrated with the current velocity update function of vehicles at each frame.
* The current velocity updates incorporate a Gauss-Markov model to simulate noise disturbances, which is typical for ocean current representations.
* CSV Database of stratified ocean current is being read and published.

#### Current Capability:
Constant ocean current
* Definable with magnitude, horizontal angle, vertical angle, time period.
* "Constant" indicates that magnitude and angles remain fixed.
Stratified (constant) ocean current
* Definable with North direction velocity [m/s], East direction velocity [rad], Depth [m]
* "constant" indicates that time period is fixed to 'always'

It is important to note that the current velocity is calculated in Gazebo's ENU reference frame since it does not recognize the SNAME convention. If you want to visualize the simulation output in RViz, you can subscribe to the topic `/<model_name>/current_velocity_marker` to see an arrow indicating the direction of the current velocity. The marker disappears if the current velocity is set to zero.
***

# Background
## Gauss-Markov model for ocean current velocity and disturbances [1]

The ocean current velocity <img src="https://render.githubusercontent.com/render/math?math=\textbf{V}_{ocean}"> is expressed as a function of magnitude, horizontal angle and vertical angle. It can be modelled using a first-order Gauss-Markov process:

<p align="center">
<img src="https://render.githubusercontent.com/render/math?math=\large \textbf{\dot{V}}_{ocean}%2B\mu \textbf{V}_{ocean}= \textbf{\omega}">
</p>


where <img src="https://render.githubusercontent.com/render/math?math=\textbf{\omega}"> is random noise and <img src="https://render.githubusercontent.com/render/math?math=\textbf{\mu} \geq 0"> (typically zero) is constant.


## Basic definitions and implementation

The [world SDF](https://github.com/uuvsimulator/uuv_simulator/blob/ddeb823a25eacfb2d7ed7569f4726881f7f289db/uuv_gazebo_worlds/worlds/ocean_waves.world#L76) consists of the following section to define ocean currents and statistics and it is self-explanatory.


```xml
    <plugin name="underwater_current_plugin" filename="libuuv_underwater_current_ros_plugin.so">
      <namespace>hydrodynamics</namespace>
      <constant_current>
        <topic>current_velocity</topic>
        <velocity>
          <mean>0</mean>
          <min>0</min>
          <max>5</max>
          <mu>0.0</mu>
          <noiseAmp>0.0</noiseAmp>
        </velocity>

        <horizontal_angle>
          <mean>0</mean>
          <min>-3.141592653589793238</min>
          <max>3.141592653589793238</max>
          <mu>0.0</mu>
          <noiseAmp>0.0</noiseAmp>
        </horizontal_angle>

        <vertical_angle>
          <mean>0</mean>
          <min>-3.141592653589793238</min>
          <max>3.141592653589793238</max>
          <mu>0.0</mu>
          <noiseAmp>0.0</noiseAmp>
        </vertical_angle>
      </constant_current>
    </plugin>
```

Here, library file `libuuv_underwater_current_ros_plugin.so` is loaded with `namespace 'hydrodynamic'` to apply to every model included in the world with `constant_current` topic. (It is also possible to specify separate namespaces for each 'model'. This might be useful if each model requires different noise.)  All models that are to be affected by the current must incorporate the `libuuv_underwater_object_ros_plugin.so` [model plugin](#UnderwaterObjectPlugin).


The Load function of [UnderwaterCurrentROSPlugin.cc](https://github.com/uuvsimulator/uuv_simulator/blob/daf088b0d6a8c282148aeba39bfc5db7dc787aa3/uuv_world_plugins/uuv_world_ros_plugins/src/UnderwaterCurrentROSPlugin.cc#40) tries to load the UnderwaterCurrentPlugin when it starts ([UnderwaterCurrentPlugin.cc](https://github.com/uuvsimulator/uuv_simulator/blob/daf088b0d6a8c282148aeba39bfc5db7dc787aa3/uuv_world_plugins/uuv_world_plugins/src/UnderwaterCurrentPlugin.cc)).


The [Load function of the UnderwaterCurrentPlugin.cc](https://github.com/uuvsimulator/uuv_simulator/blob/daf088b0d6a8c282148aeba39bfc5db7dc787aa3/uuv_world_plugins/uuv_world_plugins/src/UnderwaterCurrentPlugin.cc#56) then initializes global current velocity model variables (`currentVelModel`, `currentHorzAngleModel` and `currentVertAngleModel` which are defined as `GaussmarkovProcess` type in [UnderwaterCurrentPlugin.hh](https://github.com/uuvsimulator/uuv_simulator/blob/daf088b0d6a8c282148aeba39bfc5db7dc787aa3/uuv_world_plugins/uuv_world_plugins/include/uuv_world_plugins/UnderwaterCurrentPlugin.hh#L84)).  The "mean" values from the world SDF file are used to initialize the `var` attributes of the corresponding model variables([#118](https://github.com/uuvsimulator/uuv_simulator/blob/daf088b0d6a8c282148aeba39bfc5db7dc787aa3/uuv_world_plugins/uuv_world_plugins/src/UnderwaterCurrentPlugin.cc#L118)). The stratified current database is also loaded at this time used to initialize the stratified current variables (`stratifiedCurrentDatabase` is a vector of Vector3d objects containing the database x-velocity, y-velocity, and depth values, and `stratifiedCurrentModels` contains `GaussmarkovProcess` models for the current velocity, horizontal angle, and vertical angle at each depth).   Finally, the Load function connect the plugin's update function to the world update event and runs the following:


```c++
void UnderwaterCurrentPlugin::Update(const common::UpdateInfo & /** _info */)
{
#if GAZEBO_MAJOR_VERSION >= 8
  common::Time time = this->world->SimTime();
#else
  common::Time time = this->world->GetSimTime();
#endif

  // Calculate the flow velocity and the direction using the Gauss-Markov
  // model

  // Update current velocity
  double currentVelMag = this->currentVelModel.Update(time.Double());
  // Update current horizontal direction around z axis of flow frame
  double horzAngle = this->currentHorzAngleModel.Update(time.Double());

  // Update current horizontal direction around z axis of flow frame
  double vertAngle = this->currentVertAngleModel.Update(time.Double());

  // Generating the current velocity vector as in the NED frame
  this->currentVelocity = ignition::math::Vector3d(
      currentVelMag * cos(horzAngle) * cos(vertAngle),
      currentVelMag * sin(horzAngle) * cos(vertAngle),
      currentVelMag * sin(vertAngle));

  // Generate the depth-specific velocities
  this->currentStratifiedVelocity.clear();
  for (int i = 0; i < this->stratifiedDatabase.size(); i++)
  {
      double depth = this->stratifiedDatabase[i].Z();
      currentVelMag =
        this->stratifiedCurrentModels[i][0].Update(time.Double());
      horzAngle =
        this->stratifiedCurrentModels[i][1].Update(time.Double());
      vertAngle =
        this->stratifiedCurrentModels[i][2].Update(time.Double());
      ignition::math::Vector4d depthVel(
          currentVelMag * cos(horzAngle) * cos(vertAngle),
          currentVelMag * sin(horzAngle) * cos(vertAngle),
          currentVelMag * sin(vertAngle),
          depth);
      this->currentStratifiedVelocity.push_back(depthVel);
  }

  // Update time stamp
  this->lastUpdate = time;
  this->PublishCurrentVelocity();
  this->PublishStratifiedCurrentVelocity();
}
```

Here, at each simulation time, the global and stratified current velocity model variables(`currentVelModel`, `currentHorzAngleModel` `currentVertAngleModel`, and `stratifiedCurrentModels`) are updated and `currentVelocity` and `currentStratifiedVelocity` are calculated in the NED frame to be published.

Lastly, the `GaussmarkovProcess` type variable is updated in the following lines which implement a straightforward interpretation of the model. The max/min bounds and noise are defined in the world SDF.


```c++
double GaussMarkovProcess::Update(double _time)
{
  double step = _time - this->lastUpdate;
  double random = static_cast<double>(
    static_cast<double>(rand_r(&this->randSeed)) / RAND_MAX) - 0.5;
  this->var = (1 - step * this->mu) * this->var + this->noiseAmp * random;
  if (this->var >= this->max)
    this->var = this->max;
  if (this->var <= this->min)
    this->var = this->min;
  this->lastUpdate = _time;
  return this->var;
}
```

# Constant ocean current quickstart
1. Start standard ocean_world world without ocean current
    ```
    roslaunch dave_demo_launch dave_demo.launch
    ```
1. To set the ocean current using the  `/hydrodynamic/current_velocity` topic, type following commands in another terminal.
    ```
    rosservice call /hydrodynamics/set_current_velocity "velocity: 1.0
    horizontal_angle: -0.8
    vertical_angle: 0.2"
    ```
    or
    ```
    rosservice call /hydrodynamics/set_current_velocity "velocity: 1.0"
    rosservice call /hydrodynamics/set_current_horz_angle "angle: -0.8"
    rosservice call /hydrodynamics/set_current_vert_angle "angle: 0.2"
    ```
    After entering these commands, the REXROV vehicle will start to drift.
    * The angles are defined in Gazebo's ENU reference frame, within a range from 0 to 3.14.
    * You can plot the value using rqt_plot.
    ```
    rqt_plot /hydrodynamics/current_velocity/twist/linear
    ```
1. The same method can be used to add noise generated by the Gauss-Markov model:
    ```
    rosservice call /hydrodynamics/set_current_velocity_model "{mean: 1.0, min: 0.0, max: 2.0, noise: 0.0, mu: 0.0}"
    rosservice call /hydrodynamics/set_current_horz_angle_model "{mean: -0.8, min: -1.0, max: 0.0, noise: 0.0, mu: 0.0}"
    rosservice call /hydrodynamics/set_current_vert_angle_model "{mean: 0.2, min: 0.0, max: 0.3, noise: 0.0, mu: 0.0}"
    ```
1. Alternatively, the following commands can be used to set the current during a fix period of time:
    ```
    roslaunch uuv_control_utils set_timed_current_perturbation.launch starting_time:=0.0 end_time:=10.0 current_vel:=1.0
    horizontal_angle:=10.0 vertical_angle:=30
    ```
(Note in this case the units of angles are in degrees.)

***

## Change of ocean current
Following the tutorial above:
* set velocity, horizontal and vertical angles to 1.0, -0.8 and 0.2.
* visualize the current velocity values using rqt_plot
Now run the command below to alter the mean velocity:
```
rosservice call /hydrodynamics/set_current_velocity "velocity: 2.0"
```
You should see the change reflected in your `rqt_plot` display:
![VelocityChangeRQTPlot](https://user-images.githubusercontent.com/7955120/87672870-9702c700-c7ae-11ea-83c3-0f0bf63aafb1.png)

## Changing noise parameters
Similarly, you can dynamically alter the parameters used for generating noise. First run
```
rosservice call /hydrodynamics/set_current_velocity_model "{mean: 1.0, min: 0.0, max: 2.0, noise: 0.0, mu: 0.0}"
```
to set initial noise parameters. Next, alter these by running:
```
rosservice call /hydrodynamics/set_current_velocity_model "{mean: 1.0, min: 0.0, max: 2.0, noise: 10.0, mu: 0.0}"
```
The change will register on your `rqt_plot` display. It should looks something like this:
![NoiseChangeRQTPlot](https://user-images.githubusercontent.com/7955120/87672904-a4b84c80-c7ae-11ea-9bcf-c1ca89c1ae40.png)

### <a name="UnderwaterObjectPlugin"></a>Note: Ocean current relies on the underwater object plugin
* In order for a model to be affected by current, its SDF description must include the Underwater Object plugin (`libuuv_underwater_object_ros_plugin.so`).
* The following SDF snippet corresponds to the [Rexrov](https://github.com/Field-Robotics-Lab/dave/wiki/vehicle_examples)  UUV provided with the UUV Simulator.
* The `flow_velocity_topic` element specifies the ROS topic (including namespace) to which the plugin will subscribe for current information.
* Other elements describe the model's hydrodynamic characteristics and are documented elsewhere.


```xml
    <plugin name='uuv_plugin' filename='libuuv_underwater_object_ros_plugin.so'>
      <fluid_density>1028.0</fluid_density>
      <flow_velocity_topic>hydrodynamics/current_velocity</flow_velocity_topic>
      <debug>0</debug>
      <link name='rexrov/base_link'>
        <neutrally_buoyant>0</neutrally_buoyant>
        <volume>1.83826</volume>
        <box>
          <width>1.5</width>
          <length>2.6</length>
          <height>1.6</height>
        </box>
        <center_of_buoyancy>0.0 0.0 0.3</center_of_buoyancy>
        <hydrodynamic_model>
          <type>fossen</type>
          <added_mass>779.79 -6.8773 -103.32 8.5426 -165.54 -7.8033 -6.8773 1222 51.29 409.44 -5.8488 62.726 -103.32 51.29 3659.9 6.1112 -386.42 10.774 8.5426 409.44 6.1112 534.9 -10.027 21.019 -165.54 -5.8488 -386.42 -10.027 842.69 -1.1162 -7.8033 62.726 10.775 21.019 -1.1162 224.32</added_mass>
          <linear_damping>-74.82 -69.48 -728.4 -268.8 -309.77 -105</linear_damping>
          <quadratic_damping>-748.22 -992.53 -1821.01 -672 -774.44 -523.27</quadratic_damping>
        </hydrodynamic_model>
      </link>
      <robotNamespace>/rexrov/</robotNamespace>
    </plugin>
```

#### Video clip

[Video link (50 sec)](https://www.youtube.com/watch?v=QrcrnN9_lEo)

[![Video link (50 sec)](https://img.youtube.com/vi/QrcrnN9_lEo/0.jpg)](https://www.youtube.com/watch?v=QrcrnN9_lEo)


***

# Stratified ocean current quickstart
- Parameter for database file path location : `dave/models/dave_worlds/worlds/dave_ocean_waves_transient_current.world/<database>`
```bash
## fix the csv database file path first !
roslaunch dave_demo_launch dave_transientcurrent_demo.launch
To show the current experienced by the model: `rostopic echo /hydrodynamics/current_velocity/rexrov/twist/linear`, here `rexrov` is the name of the vehicle.
```
If you are familiar with rviz configurations `/<model_name>/current_velocity_marker` will make an arrow indicating ocean current

## Configuration file locations
- `world plugin` configuration :
   - `dave/models/dave_worlds/worlds/dave_ocean_waves_transient_current.world`
```xacro
    <plugin name="underwater_current_plugin" filename="libdave_ocean_current_plugin.so">
      <namespace>hydrodynamics</namespace>
      <transient_current>
        <topic_stratified>stratified_current_velocity</topic_stratified>
        <!-- Database tag can accept full path or filename for .csv within the dave_worlds/worlds folder  -->
        <databasefilePath>transientOceanCurrentDatabase.csv</databasefilePath>
      </transient_current>
```
- `model plugin` configuration
     - `dave/urdf/robots/rexrov_description/urdf/rexrov_oberon7_transient_current.xacro`

```xacro
     <plugin name="dave_transient_current_plugin" filename="libdave_ocean_current_model_plugin.so">
        <flow_velocity_topic>hydrodynamics/current_velocity/$(arg namespace)</flow_velocity_topic>
        <base_link_name>$(arg namespace)/base_link</base_link_name>
        <transient_current>
          <topic_stratified>stratified_current_velocity</topic_stratified>
```

- Stratified ocean current database
   - 3 columns, North direction velocity [m/s], East direction velocity [m/z], Depth [m]
   - Database tag can accept full path or filename for .csv within the dave/models/dave_worlds/worlds folder

## Publishing ROS msgs
- `/hydrodynamics/stratified_current_velocity`
   - A timestamped series of depths and corresponding current velocities in the NED frame.
   - [dave_gazebo_ros_plugins::StratifiedCurrentVelocity.msg](https://github.com/Field-Robotics-Lab/dave/blob/master/gazebo/dave_gazebo_ros_plugins/msg/StratifiedCurrentVelocity.msg) format timestamped array of depths and array of corresponding `geometry_msgs::Vector3` velocites.
- `/hydrodynamics/stratified_current_velocity_database`
   - Stratified current values (depths and velocities in the NED frame) and tidal velocities read from the stratified current and tidal oscillation databases.
   - [dave_gazebo_ros_plugins::StratifiedCurrentDatabase.msg](https://github.com/Field-Robotics-Lab/dave/blob/master/gazebo/dave_gazebo_ros_plugins/msg/StratifiedCurrentDatabase.msg) format array of depths, array of corresponding `geometry_msgs::Vector3` velocites, and tidal oscillation data.
 - `/hydrodynamics/current_velocity/<model name>`
   - A single 3 dimensional vector of velocities in vehicle body frame (1 body only)
   - Generic `geometry_msgs::TwistStamped` format. X, Y, Z velocities saved in twist.linear.x, twist.linear.y, twist.linear.z.


## Notes
- Modified from `uuv_world_plugin` of `uuv_simulator`
- The original plugins are structured as `world plugin` defined at `.world` file. In order to obtain vehicle depth, new `model plugin` is added to be defined at each model `xacro` file.
- The database (table, csv file) is located at a directory alongside world files and its path configured at world file for `world plugin`
- The base_link name is required at `model plugin` which is defined at each model `xacro` file.

## Basic diagram for plugin structures
![PluginDiagram](https://user-images.githubusercontent.com/7955120/95210955-f826bd80-0826-11eb-8c86-d78143ebfbbe.png)


# Stratified Ocean current plugin with multiple vehicle support

## Quick run command
```
# Run each roslaunch at separate terminal windows
roslaunch dave_demo_launch dave_transient_current_demo.launch
roslaunch dave_demo_launch dave_transient_current_multi.launch
```
* If using dockwater (Docker environment), launch first terminal using `./run.bash noetic:latest` and second terminal using `./join.bash noetic_runtime`.
## Code structure
![image](https://user-images.githubusercontent.com/7955120/109739573-ab010a80-7b7e-11eb-8745-6999a782c957.png)

- dave_world_plugins
  - The plugin is loaded when dave_world_ros_plugins is loaded
  - To read `<constant_current>` parameters and publish global ocean current Gazebo topic
  - To read stratified ocean current database CSV file and save it to the world
- dave_world_ros_plugins
  - The plugin is loaded at `uuv_dave_ocean_waves_transientcurrent.world`
  - To relay global ocean current to ROS topic
  - To relay stratified ocean current database to ROS topic with custom msg format defined at dave_world_ros_plugins_msgs/msg/StratifiedCurrentVelocity.msg
- dave_model_plugins
  - The plugin is loaded at uuv_dave/urdf/rexrov_oberon7_transient_current.xacro
  - To read stratified ocean current database ROS topic and calculate ocean current relative to vehicle depth
  - To publish vehicle-wise ocean current Gazebo topic
  - To publish vehicle-wise ocean current ROS topic

The hydrodynamics plugin (we are using the uuv_simulator plugin which is loaded at uuv_dave/urdf/rexrov_oberon7_transient_current.xacro) uses Gazebo topic to apply hydrodynamics calculations for each vehicle. Not sure if we can have dave_model_plugins publish only ROS topic and make it work for the hydrodynamics plugin (Duane advised me not to have a plugin to publish both ROS and Gazebo topics before).

## Multiple vehicle support how-to
A vehicle being affected by the ocean current is defined at hydrodynamics plugins with `<flow_velocity_topic>` parameter which is at uuv_ws/src/dave/urdf/robots/rexrov_descripton/urdf/rexrov_oberon7_transient_current.xacro. Having it to a vehicle-specific topic, here it is hydrodynamics/current_velocity/rexrov, a vehicle will be affected by it.
```XML
<plugin name="uuv_plugin" filename="libuuv_underwater_object_ros_plugin.so">
  <flow_velocity_topic>hydrodynamics/current_velocity/$(arg namespace)</flow_velocity_topic>
</plugin>
```
The vehicle-specific topic is published using the dave_model_plugins (dave_transient_current_plugin) which is also at uuv_ws/src/dave/urdf/robots/rexrov_descripton/urdf/rexrov_oberon7_transient_current.xacro
```XML
<plugin name="dave_transient_current_plugin" filename="libdave_transient_current_plugin.so">
  <flow_velocity_topic>hydrodynamics/current_velocity/$(arg namespace)</flow_velocity_topic>
  <base_link_name>$(arg namespace)/base_link</base_link_name>
  <transient_current>
    <topic_stratified_database>stratified_current_velocity_database</topic_stratified_database>
    <velocity_north>
      <noiseAmp>0.3</noiseAmp>
      <noiseFreq>0.0</noiseFreq>
    </velocity_north>
    <velocity_east>
      <noiseAmp>0.3</noiseAmp>
      <noiseFreq>0.0</noiseFreq>
    </velocity_east>
    <velocity_down>
      <noiseAmp>0.3</noiseAmp>
      <noiseFreq>0.0</noiseFreq>
    </velocity_down>
  </transient_current>
</plugin>
```
Here, the `<flow_velocity_topic>` should match with the topic that the hydrodynamics plugin is subscribing to. Also, `<topic_stratified_database>` should match with the ROS database topic at dave_world_ros_plugins which is defined in the world file.


## Multiple vehicle example
- Two vehicles (Rexrov and Smilodon) effective by ocean currents at each vehicle's depths.


To demonstrate the functionality of the stratified current capability with multiple vehicles, launch the transient current demo:
```
roslaunch dave_demo_launch dave_transient_current_demo.launch
```
Once the simulation launches successfully, add the second vehicle:
```
roslaunch dave_demo_launch dave_transient_current_multi.launch
```
You can observe the depth-dependent variation on the ROS current topics via:
```
rostopic echo /hydrodynamics/current_velocity/smilodon
rostopic echo /hydrodynamics/current_velocity/rexrov
```
Smilodon is under faster ocean currents than Rexrov according to the database provided. (slower ocean current near seabed)

![image](https://user-images.githubusercontent.com/7955120/110289016-b056be80-7f9d-11eb-9867-47e49b0abcc1.png)

# Tidal Oscillation Support
![image](https://user-images.githubusercontent.com/7955120/110070352-a7ac8100-7d2e-11eb-801b-8afd226124fa.png)
- EPIC glider is moving at around 20~80 cm/sec. Tidal oscillation of the ocean currents are about 40 cm/s at flood and -30 cm/s (opposite direction) at ebb.
- This new set of codes (`TidalOscillation.hh` and `TidalOscillation.cc`) are designed to incorporate the tidal oscillation
- It's uniform for the X-Y dimension. Z dimension (depth) dependencies can be applied using transientOceanCurrentDatabase.csv as before.
- It does not work as a global ocean current but it supports multiple vehicles individually.

## Configuration parameters
### Input data
- NOAA Annual prediction data
   - The data can be downloaded from [NOAA website](https://tidesandcurrents.noaa.gov/noaacurrents/Annual?id=ACT1951_1). They should be downloaded in CSV, cm/sec, 24-hour units format.
-  Single-dimension stratified ocean current (depth vs velocity) to express amplitude ratio
   - Same transientOceanCurrentDatabase.csv format is used. But only takes the first column (north direction current) as a ratio depending on the depth. Ratio with 1.0 as a default ocean current interpolated from NOAA data.

### How to configure
- World file
https://github.com/Field-Robotics-Lab/dave/blob/602c1e1ba22fddf5cb960993de1858818a96523d/uuv_dave/worlds/uuv_dave_ocean_waves_transientcurrent.world#L114-L159
   - Tidal amplitudes (Two methods available)
     - Method 1 (Database file)
        - databaseFilePath : file path for the CSV database file downloaded from  [NOAA current prediction](https://tidesandcurrents.noaa.gov/noaacurrents/Predictions?id=ACT1951_1).
     - Method 2 (Harmonic constituents)
        - Tide/water level harmonic constituents of NOAA is used (https://tidesandcurrents.noaa.gov/harcon.html?unit=0&timezone=0&id=8447685&name=Chappaquoit+Point&state=MA)
        - First three constituents are adopted with 90 degrees phase shifts
   - mean_direction : mean flood and ebb current direction from NOAA
   - world_start_time_GMT : starting time of the simulation in GMT.
- Model SDF
https://github.com/Field-Robotics-Lab/dave/blob/2e1eb1dc3a33f54ee9c25e93bde119c45ada2b43/uuv_dave/urdf/rexrov_oberon7_transient_current.xacro#L100




### References

[1] [T. I. Fossen, “Handbook of Marine Craft Hydrodynamics and Motion Control,” Apr. 2011.](http://clairedune.kippou.fr/Fossen.pdf). (pp. 223-4)



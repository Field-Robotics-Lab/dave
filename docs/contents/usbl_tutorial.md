---
layout: default
title: Ultra Short Baseline (USBL) Gazebo Plugin Tutorial
nav_exclude: true
---

# Ultra Short Baseline (USBL) Gazebo Plugin Tutorial

USBL is used to assist underwater navigation and positioning. It consists of one transceiver and one or multiple transponders (or beacons). Usually, the transceiver will be attached to either a stationary object or mobile central node, whereas the transponder(s) is attached to objects that are being tracked. For more information, see [this] video (best explanation I have seen so far).

## Quick Start

The tutorial below describes how to generate this demo, but for convenience the world and launch  files for the demo are included for demonstration and testing purposes.

Launch the tutorial:
```
roslaunch dave_sensor_launch usbl_tutorial.launch verbose:=true
```
Gazebo will be launch and one may see the following scene

![](./images/tutorial_world.png)


In this tutorial, the box acts as transceiver, whereas the sphere acts as transponder.

Upon the world creation, the following ROS topics will be created:
```
/USBL/common_interrogation_ping
/USBL/transceiver_manufacturer_168/channel_switch
/USBL/transceiver_manufacturer_168/command_response
/USBL/transceiver_manufacturer_168/interrogation_mode
/USBL/transceiver_manufacturer_168/transponder_location
/USBL/transceiver_manufacturer_168/transponder_location_cartesian
/USBL/transponder_manufacturer_1/command_request
/USBL/transponder_manufacturer_1/individual_interrogation_ping
```

Echo the position of the transponder as reported by the transceiver
```
 rostopic echo /USBL/transceiver_manufacturer_168/transponder_location
```
OR
```
rostopic echo /USBL/transceiver_manufacturer_168/transponder_location_cartesion
```

If a faster ping rate is desired, one can decrease the value of the `<ping_freq>` tag in the `tutorial.world` file.


Command the transponder to ping
```
rostopic pub --once /USBL/common_interrogation_ping std_msgs/String "data: 'ping'"
```

The message type used in representing the coordinates is in `geometry_msgs/Vector3`. To avoid confusion, the following table provides some clarities Vector3 interpretation in each coordinate system:

| Vector3 msg fields | Spherical coordinates | Cartesian coordinates |
| ------   | ------  |------ |
| x | Bearing | X
| y | Range | Y
| z | Elevation | Z |

## Tutorial

### Creating world with at least two models
Because the USBL plugin is model plugin, it can be attached to any model. For example, we can create a empty world with a box model and a sphere model:
```
<?xml version="1.0"?>
<gazebo version="1.2">
    <world name="default">
        <!-- <include filename="ground_plane.model"/>
        <include filename="sun.light"/> -->
        <include>
            <uri>model://ground_plane</uri>
        </include>

        <include>
            <uri>model://sun</uri>
        </include>

        <model name="box">
            <pose>0 0 0.5 0 0 0</pose>
            <link name="link">
                <collision name="collision">
                    <geometry>
                        <box>
                          <size>1 1 1</size>
                        </box>
                    </geometry>
                </collision>

                <visual name="visual">
                    <geometry>
                        <box>
                          <size>1 1 1</size>
                        </box>
                    </geometry>
                </visual>
            </link>
        </model>

        <model name="sphere">
            <pose>3 3 0.5 0 0 0</pose>
            <link name="link">
                <collision name="collision">
                    <geometry>
                        <sphere>
                            <radius>1</radius>
                        </sphere>
                    </geometry>
                </collision>

                <visual name="visual">
                    <geometry>
                        <sphere>
                            <radius>1</radius>
                        </sphere>
                    </geometry>
                </visual>
            </link>
        </model>
    </world>
</gazebo>
```

Then, right after the box link end tag, add the USBL transceiver plugin:
```
<plugin name="usbl_transceiver" filename="libtransceiverPlugin.so">
    <namespace>USBL</namespace>
    <transponder_device>trasnponder_manufacturer</transponder_device>
    <transponder_ID>1</transponder_ID>
    <transceiver_device>tranceiver_manufacturer</transceiver_device>
    <transceiver_ID>168</transceiver_ID>
    <enable_ping_scheduler>false</enable_ping_scheduler>
    <transponder_attached_object>sphere</transponder_attached_object>
</plugin>
```
The following table explains what each tag in USBL transceiver plugin means. Most of the tags are required to load the plugin properly, while optional tags will be otherwise mentioned in the Notes column. Same requirement is also applied to USBL transponder plugin.
| Tag name | Meaning | Notes |
| ------   | ------  |------ |
| namespace | USBL namespace |
| transponder_device | Transponder model | can be manufactuerer or vendor name
| transponder_ID | Unique trasnponder ID | can be multiple IDs separated with comma |
| transceiver_device | Transceiver model | can be manufactuerer or vendor name
| transceiver_ID | Unique transceiver ID |
| enable_ping_scheduler | Enable the functionality of sending ping every second  | 0 or 1|
| transponder_attached_object| Gazebo model that transponder attaches to |
| interrogation_mode | Determines how transceiver sends ping to transponder | Optional, choose between common (default) or individual

After the sphere link end tag, add the USBL transponder plugin:
```
<plugin name="usbl_transponder" filename="libtransponderPlugin.so">
    <namespace>USBL</namespace>
    <transponder_device>trasnponder_manufacturer</transponder_device>
    <transponder_ID>1</transponder_ID>
    <transceiver_device>tranceiver_manufacturer</transceiver_device>
    <transceiver_ID>168</transceiver_ID>
    <mu>0</mu>
    <sigma>0.0</sigma>
</plugin>
```
The following table explains what each tag in USBL transponder plugin means.
| Tag name | Meaning | Notes |
| ------   | ------  |------ |
| namespace | USBL namespace |
| transponder_device |Ttransponder model | Can be manufactuerer or vendor name
| transponder_ID | Unique trasnponder ID | Single ID |
| transceiver_device | Transceiver model | Can be manufactuerer or vendor name
| transceiver_ID | Unique transceiver ID |
| mu | mean of noise  | Optional, default is 0.0
| sigma| standard deviation of noise | Optional, default is 1.0

When the plugins are placed within the model, the following ROS topic will be appeared:
```
/USBL/common_interrogation_ping
/USBL/tranceiver_manufacturer_168/channel_switch
/USBL/tranceiver_manufacturer_168/command_response
/USBL/tranceiver_manufacturer_168/command_response_test
/USBL/tranceiver_manufacturer_168/interrogation_mode
/USBL/tranceiver_manufacturer_168/temperature
/USBL/tranceiver_manufacturer_168/transponder_location
/USBL/tranceiver_manufacturer_168/transponder_location_cartesion
/USBL/transponder_manufacturer_1/command_request
/USBL/transponder_manufacturer_1/individual_interrogation_ping
/USBL/transponder_manufacturer_1/temperature
```

To verify that both USBL transceiver and transponder function properly, one can publish ping signal on another terminal window:
```
rostopic pub --once /USBL/common_interrogation_ping std_msgs/String "data: 'ping'"
```
then echo the tracked location



Transceiver then reports the location of the transponder's position in both spherical and Cartesian coordinates base on transceiver's frame of reference.
```
rostopic echo /USBL/tranceiver_manufacturer_168/transponder_location
```

[this]: <https://www.youtube.com/watch?v=ZYTqp2thhZA&ab_channel=Sonardyne>


### How does USBL Gazebo Plugin work
There are at least two entities involved in USBL operation, one transceiver and one or more trasnponders. In Gazebo simulation environment, all transponders deployed will subscribe to `common_interrogation_ping` and their corresponding `individual_interrogation_ping`. The overall communication is described in the following picture:

![](./images/interrogation_modes.png)

A ping signal can be sent by publishing a string message with data field `ping` to either `common_interrogation_ping` topic or `individual_interrogation_ping` topic (depending on transceiver's communication channel). If the intention is to gather position for all transponders, `common_interrogation_ping` is a more appropriate topic choice.

If the user wants to ping a particular transponder, individual interrogation mode must be used. To switch to individual interrogation mode, one can publish string msg to `interrogation_mode` topic. For example,

```
rostopic pub --once /USBL/transceiver_manufacturer_168/interrogation_mode std_msgs/String "data: 'individual'"
```

Currently, only `common` and `individual` mode are supported. The channel on the transceiver must match the transponder ID. To switch channel on the transceiver, one can simply publish a string message with data `<tranponderID>` to `channel_switch` topic at the transceiver side. For example,

```
rostopic pub --once /USBL/transceiver_manufacturer_168/channel_switch std_msgs/String "data: '1'"
```

Then publish a message to `individual_interrogatoin_ping` topic of the target transponder.
```
rostopic pub --once /USBL/transponder_manufacturer_1/individual_interrogation_ping std_msgs/String "data: 'hello world'"
```

The overview of the ros node communication is depicted below:

![](./images/rosgraph_usbl.png)


### Parameters of the Plugins
In tutorial.world file, a world is created with two models: a box at the center and a sphere at (3, 3), as one may see in the scene at the beginning of the tutorial. Because both plugins are currently model plugin, they should be placed within the model block. In the case of the transceiver plugin, one may see the following inside the box model in the tutorial.world file.

```
<plugin name="usbl_transceiver" filename="libtransceiverPlugin.so">
    <namespace>USBL</namespace>
    <transponder_device>trasnponder_manufacturer</transponder_device>
    <transponder_ID>1</transponder_ID>
    <transceiver_device>tranceiver_manufacturer</transceiver_device>
    <transceiver_ID>168</transceiver_ID>
    <enable_ping_scheduler>false</enable_ping_scheduler>
    <transponder_attached_object>sphere</transponder_attached_object>
    <interrogation_mode>common</interrogation_mode>
    <sound_speed>1540.0</sound_speed>
    <ping_freq>2</ping_freq>
</plugin>
```
The following table explains what each tag in USBL transceiver plugin means. Most of the tags are required to load the plugin properly, while optional tags will be otherwise mentioned in the Notes column. Same requirement is also applied to USBL transponder plugin.
| Tag name | Meaning | Notes |
| ------   | ------  |------ |
| namespace | USBL namespace |
| transponder_device | Transponder model | can be manufactuerer or vendor name
| transponder_ID | Unique trasnponder ID | can be multiple IDs separated with comma |
| transceiver_device | Transceiver model | can be manufactuerer or vendor name
| transceiver_ID | Unique transceiver ID |
| transponder_attached_object| Gazebo model that transponder attaches to |
| interrogation_mode | Determines how transceiver sends ping to transponder | Optional, choose between common (default) or individual
| sound_speed| speed of the sound in ocean | default is 1540.4 m/s
| ping_freq | The frequency of sending pings | default is 1Hz with common interrogation mode

<br>

In the case of the transponder (inside the sphere block in tutorial.world):
```
<plugin name="usbl_transponder" filename="libtransponderPlugin.so">
    <namespace>USBL</namespace>
    <transponder_device>trasnponder_manufacturer</transponder_device>
    <transponder_ID>1</transponder_ID>
    <transceiver_device>tranceiver_manufacturer</transceiver_device>
    <transceiver_ID>168</transceiver_ID>
    <sound_speed>1540.0</sound_speed>
    <mu>0</mu>
    <sigma>0.0</sigma>
</plugin>
```
The following table explains what each tag in USBL transponder plugin means.
| Tag name | Meaning | Notes |
| ------   | ------  |------ |
| namespace | USBL namespace |
| transponder_device |Ttransponder model | Can be manufactuerer or vendor name
| transponder_ID | Unique trasnponder ID | Single ID |
| transceiver_device | Transceiver model | Can be manufactuerer or vendor name
| transceiver_ID | Unique transceiver ID |
| sound_speed| speed of the sound in ocean | default is 1540.4 m/s
| mu | mean of noise  | Optional, default is 0.0
| sigma| standard deviation of noise | Optional, default is 1.0

<br>

Note: If one wants to get exact location of the transponder(s), `mu` and `sigma` should be zero.


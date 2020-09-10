# Ultra Short Baseline (USBL) Gazebo Plugin Tutorial

USBL is used to assist underwater navigation and positioning. It consists of one transceiver and one or multiple transponders (or beacons). Usually, the transceiver will be attached to either a stationary object or mobile central node, whereas the transponder(s) is attached to objects that are being tracked. For more information, see [this] video (best explanation I have seen so far). 

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

After the box link end tag, add the USBL transceiver plugin:
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
/USBL/trasnponder_manufacturer_1/command_request
/USBL/trasnponder_manufacturer_1/individual_interrogation_ping
/USBL/trasnponder_manufacturer_1/temperature
```

To verify that both USBL transceiver and transponder function properly, one can publish ping signal on another terminal window:
```
rostopic pub --once /USBL/common_interrogation_ping std_msgs/String "data: 'ping'"
```

Transceiver then reports the location of the transponder's position in both spherical and Cartesian coordinates base on transceiver's frame of reference.


[this]: <https://www.youtube.com/watch?v=ZYTqp2thhZA&ab_channel=Sonardyne>
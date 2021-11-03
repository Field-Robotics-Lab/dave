# Custom model properties

Custom model properties, achieved by
[custom elements and attributes](http://sdformat.org/tutorials?tut=custom_elements_attributes_proposal),
are useful for properties not captured by typical sensors, which can be picked
up in a custom plugin.
For example, physical properties like surface material, roughness, slippage,
and other properties affect the signal strength reflected back to multi-beam
sonars.
Such properties are neither captured by the standard SDF specification nor
typical sensors, and are application-dependent.

Users may choose to write custom sensors or plugins to make use of such
custom properties.

The advantage of specifying these properties in the SDF is to keep them separate
from the plugin code, close to the object, and be associated with specific
objects.

## Dependencies

[Ruby](https://www.ruby-lang.org) is required:
```
sudo apt-get install -y ruby
```

## Generate SDF world from ERB template

This uses ERB templates to generate SDF files.
[An introductory tutorial](https://ignitionrobotics.org/api/gazebo/7.0/erbtemplate.html)
can be found in Ignition Gazebo.

In the ERB file, Ruby commands are wrapped in `<% %>` angle brackets and
inserted into regular SDF XML.
```
erb coke_template.erb > coke.sdf
```

An example result is provided in a reference file `coke.ref.sdf`.

Specific to this repository's directory structure, to easily find the file in
ROS, you may want to export the file elsewhere, such as
```
erb coke_template.erb > ../../../models/dave_worlds/worlds/distorted_coke.world
```

## Load the SDF in Gazebo

An example launch file is provided, which loads the SDF world:
```
roslaunch dave_demo_launch distorted_coke.launch 
```

## Resources

For parameters in the `<collision>` tag, see [`sdformat` specification](http://sdformat.org/spec?ver=1.8&elem=collision).

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

[Ruby](https://www.ruby-lang.org) is required for generating SDF files from
ERB templates:
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

## Custom SDF elements for surface properties

[An introductory tutorial](http://sdformat.org/tutorials?tut=custom_elements_attributes_proposal&cat=pose_semantics_docs&)
on custom SDF elements and attributes can be found on the sdformat website.

### Schema

Currently, these custom properties are defined:

- `<surface_props:material>`: `string`
- `<surface_props:distort_extent>`: `float`, in range `[0, 1]`.
- `<surface_props:roughness>`: `double`, in range `[0.0, 1.0]`

Example SDF snippet:
```
<surface_props:material>metal</surface_props:material>
<surface_props:distort_extent>30</surface_props:distort_extent>
<surface_props:roughness>0.3</surface_props:roughness>
```

### Custom Gazebo plugin

The custom SDF tags are handled by a custom C++ Gazebo plugin that reads the SDF
file.
An example plugin (`custom_surface_properties` in `dave_gazebo_model_plugins`
package) is provided, which reads the custom SDF tags.

This plugin is loaded in the SDF file.

### To add new custom tags

To add a new custom SDF tag or change an existing one, follow these steps:
1. In your SDF file (or ERB file, if using one to generate the SDF), add a tag,
   following the pattern in the schema above.
2. In your C++ Gazebo plugin, add a call to the SDF parser to read the new tag.
   See the plugin above for example calls.

## Load the SDF in Gazebo

An example launch file is provided, which loads the SDF world generated above:
```
roslaunch dave_demo_launch distorted_coke.launch 
```

## Resources

For parameters in the `<collision>` tag, see [`sdformat` specification](http://sdformat.org/spec?ver=1.8&elem=collision).

In Gazebo 11, regardless of the physics engine, parameters in the `<ode>` tag
are used for most engines.
This is an implementation detail.
More documentation upstream is required, as ticketed in [this issue](https://github.com/ignitionrobotics/sdformat/issues/31).

## Future work

- Write a regression test to diff between the ERB generated file and the
  reference SDF file in the repo.

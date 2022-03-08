---
layout: default
title: Object Degradation Tool
parent: Underwater Object Models
grand_parent: Dave Models
nav_order: 2
---

## 3D mesh model distortion

Geometric distortions of 3D models may be desired for many use cases, such as
to test the generalization of machine learning algorithms for manipulation.

One way to programmatically distort models is to use Blender Python scripting.
Blender is a free open source 3D modeling software used by professional artists
for games and films.
It offers a Python API for almost anything available in its GUI.

This gives us the capability to quickly distort object models to incremental
extents, to be used for comparisons and evaluations of algorithms that use the
object models in applied scenarios.

See the README
[here](https://github.com/Field-Robotics-Lab/dave/blob/273a2465f1a8566b015d58dc361ad225167f39e8/urdf/scripts/mesh_distortion/README.md)
for documentation and tutorial.

## Custom SDF model properties

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

See the README
[here](https://github.com/Field-Robotics-Lab/dave/blob/273a2465f1a8566b015d58dc361ad225167f39e8/urdf/scripts/sdf_custom_properties/README.md)
for documentation and tutorial.

## Future work

We plan on upstreaming these tools to OSRF repositories as usage examples.

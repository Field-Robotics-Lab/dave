# Model distortion

Distortions on 3D models may be desired for many use cases, such as to test
the generalization of machine learning algorithms.

## Prerequisites

These instructions have been tested in [Blender](https://www.blender.org/) 2.92,
which has a revamped user interface from previous versions.
The version in `apt-get` may be older.
These instructions should work in newer versions.
Install manually as needed.
If newer versions do not work for you, Blender 2.92 can be found
[here](https://download.blender.org/release/Blender2.92).

Helpful tip for Blender Python development:
To show Python API in the tooltips when the cursor is hovered over a button or
field, go to Edit > Preferences, Interface tab, Display group, check Python
Tooltips.

## Usage

Launch Blender GUI.

### Locate model file and mesh name

Before we run the script, we need to locate the path of the 3D model file and
find the name of the part in the actual model file.
These will be passed as parameters to the script.

First, open or import the 3D model file of the object you want to distort.
In upper-left corner, File > Import.
Navigate to the mesh file you want.
There are two Coke can models as examples:
`models/dave_object_models/models/Coke/meshes/coke.obj` and
`models/dave_object_models/models/Coke_Can/meshes/coke_can.obj`.
Choose Transform while importing: Z Up, X Forward.

Tip: To view the textured object in the viewport, click on the globe icon in the
upper right of the viewport, which is for Viewport Shading: Material Preview.

If the texture is not loading correctly for you, check that you have the entire
folder for the object, which includes the `meshes` and `materials` directories.

Look for the mesh name in the model.
You can find this in the View Layer list in the upper-right panel.
If the model has multiple meshes, you will need to merge the parts, so that the
entire model is distorted, or choose the part that you wish to distort.

### Tweak the model (optional)

Note that depending on how the polygons are arranged on a mesh, the results of
the distortion will differ.
Advanced users may choose to split up the polygons or otherwise change the
polygon layout to get more even distortion results.
This is a matter of trial and error until you are happy with the distortion
result.

### Run the script

At the top of Blender GUI, go to the Scripting tab.
All commands will be executed in the Console panel in the middle-left of the
screen.

Define the full path to the model file and the mesh name, which you found above.
For example, the `Coke` model:
```
file_path = '/path/to/dave/models/dave_object_models/models/Coke/meshes/coke.obj'
object_prefix = 'LPCoke_Cube'
```

Or the `Coke_Can` model:
```
file_path = '/path/to/dave/models/dave_object_models/models/Coke_Can/meshes/coke_can.obj'
object_prefix = 'coke_can'
```

Set optional arguments. If not specified, the default will be used.
Make sure `method` is specified as a list in brackets, even if it contains only
one element.
```
distort_extent = 0.2
method = ['subdiv_mod', 'vert_rand', 'edge_subdiv']
```

Put the args into the input array:
```
import sys
sys.argv = ['distort.py', file_path, object_prefix, distort_extent, method]
```

Run the script, replacing its path with the one on your machine:
```
exec(open('/path/to/dave/urdf/scripts/mesh_distortion/distort.py').read());
```

This will execute the script with the command line arguments defined in
`sys.argv` and export the result to file.

## To add a new distortion method

There are many ways in Blender to modify a mesh.
The example methods in the script may not have everything you need.

To add a new distortion method to the script, follow these steps:
1. Experiment with the distortion method you want, manually in Blender, until
   you know the exact steps to reproduce a desired result.
   Find the Python API for the fields and buttons in the Blender user interface.
   You can find the Blender Python API name by hovering on most buttons, if
   you have Python Tooltips turned on in Edit > Preferences.
2. In the Blender Python script, add a string parameter describing the
   distortion type to the `METHODS` list.
   A descriptive name would be one that closely resembles the core essential
   function name in Blender.
3. Add a Python function to perform all the necessary steps programmatically,
   using the API you found above.
4. Following the existing pattern in the Python script, add an if-statement to
   call the new method.
5. Run it!
   Trial and error to make sure it works for different models.
   See troubleshoot section below.

## Troubleshoot

### Incorrect context

Blender might complain about incorrect context.
One cause is that the object you are trying to modify is not the "active"
object.
(Note that this is different from the object being selected.
An object can be selected, indicated by an orange outline, but not active,
which would be indicated by a white outline.)

To make the object active, find the object programatically, then call
```
bpy.context.view_layer.objects.active = obj
```

There are example calls in the Python script.

## Reference: Fouling rating scale

The script currently does not follow this scale.
We do not have active plans to adhere to this scale.
It is reproduced here for reference for possible adaptations.

Reproduced from Table 081-1-1 in Waterborne Underwater Hull Cleaning of Navy
Ships, Chapter 081, Naval Ships' Technical Manual, S9086-CQ-STM-010,
revision 5, 1 Oct 2006.

Fouling ratings (FR) in order of increasing severity

Type | FR | Description 
:---: | :---: | ---
Soft | 0 | A clean, foul-free surface; red and/or black AF paint or a bare metal surface.
Soft | 10 | Light shades of red and green (incipient slime). Bare metal and painted surfaces are visible beneath the fouling.
Soft | 20 | Slime as dark green patches with yellow or brown colored areas (advanced slime). Bare metal and painted surfaces may by obscured by the fouling.
Soft | 30 | Grass as filaments up to 3 inches (76 mm) in length, projections up to 1/4 inch (6.4 mm) in height; or a flat network of filaments, green, yellow, or brown in color; or soft non calcareous fouling such as sea cucumbers, sea grapes, or sea squirts projecting up to 1/4 inch (6.4 mm) in height. The fouling can not be easily wiped off by hand.
Hard | 40 | Calcareous fouling in the form of tubeworms less than 1⁄4 inch in diameter or height.
Hard | 50 | Calcareous fouling in the form of barnacles less than 1⁄4 inch in diameter or height.
Hard | 60 | Combination of tubeworms and barnacles, less than 1⁄4 inch (6.4 mm) in diameter or height.
Hard | 70 | Combination of tubeworms and barnacles, greater than 1⁄4 inch in diameter or height.
Hard | 80 | Tubeworms closely packed together and growing upright away from surface. Barnacles growing one on top of another, 1⁄4 inch or less in height. Calcareous shells appear clean or white in color.
Hard | 90 | Dense growth of tubeworms with barnacles, 1⁄4 inch or greater in height; Calcareous shells brown in color (oysters and mussels); or with slime or grass overlay.
Composite | 100 | All forms of fouling present, Soft and Hard, particularly soft sedentary animals without calcareous covering (tunicates) growing over various forms of hard growth.

## Tips

- Collision geometry

  When using the mesh with a physics engine, for example by way of an SDF file
  to be loaded into Gazebo, note that it is usually not a good idea to use the
  visual mesh for collision, because of the high polygon count.
  Typically, a simple primitive is used for collision, which makes computations
  much faster.

  However, if the goal is to have mesh deformation affect physical interactions
  like manipulation, using a simple primitive would defeat the purpose.
  In that case, a good compromise is to optimize (for example, using the
  Decimate modifier in Blender) the visual mesh down to a fraction of its
  polygon count, and use the optimized mesh for collision.
  There will be mismatches between collision and visual geometry, but collision
  computations would be much faster.

## Known issues

- Blender sometimes does not export the texture back to a COLLADA file
  correctly.
  Importing OBJ and exporting COLLADA works fine.

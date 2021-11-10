# Model distortion

Distortions on 3D models may be desired for many use cases, such as to test
the generalization of machine learning algorithms.

## Prerequisites

These instructions are written for [Blender](https://www.blender.org/) 2.92,
which has a revamped user interface.
The version may be newer than the version in `apt-get`.
Install manually as needed.

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

Look for the mesh name in the model.
If the model has multiple parts, you will need to merge the parts, so that the
entire model is distorted, or choose the part that you wish to distort.

Note that depending on how the polygons are arranged on a mesh, the results of
the distortion will differ.
Advanced users may choose to split up the polygons or otherwise change the
polygon layout to get more even distortion results.

### Run the script

At the top of Blender GUI, go to the Scripting tab.
All commands will be executed in the Console panel in the middle-left of the
screen.

Define the path to the model file and the mesh name, which you found above.
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
```
fouling_rating = 20
method = ['subdiv_mod', 'vert_rand', 'edge_subdiv']
```

Put the args into the input array:
```
import sys
sys.argv = ['distort.py', file_path, object_prefix, fouling_rating, method]
```

Run the script, replacing its path with the one on your machine:
```
exec(open('/path/to/dave/urdf/scripts/mesh_distortion/distort.py').read());
```

This will execute the script with the command line arguments defined in
`sys.argv` and export the result to file.

## Fouling rating scale

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
  Inputting OBJ and exporting COLLADA works fine.

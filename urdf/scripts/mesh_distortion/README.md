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

## Known issues

- Blender sometimes does not export the texture back to a COLLADA file
  correctly.
  Inputting OBJ and exporting COLLADA works fine.

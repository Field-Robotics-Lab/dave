# Model distortion

Distortions on 3D models may be desired for many use cases, such as to test
the generalization of machine learning algorithms.

## Prerequisites

The scripts are tested with [Blender](https://www.blender.org/) 2.9.2,
which may be newer than the version in `apt-get`.
Install manually as needed.

Helpful tip for Blender Python development:
To show Python API in the tooltips when the cursor is hovered over a button or
field, go to Edit > Preferences, Interface tab, Display group, check Python
Tooltips.

## Usage

Launch Blender GUI.

At the top, go to the Scripting tab.

In upper-left corner, File > Import.
Navigate to the mesh file, e.g. `models/dave_object_models/models/Coke/meshes/coke.obj`.
For this specific file, choose Transform while importing: Z Up, X Forward.

Right-click a mesh in the 3D view panel to select it.

In the console panel in the middle of the left of the screen, set the mesh name
argument:
```
file_path = '/path/to/file.dae'
object_name = 'LPCoke_Cube.000'
```

Set optional arguments:
```
fouling_rating = 10
method = 'deform'
```

Put the args into the input array:
```
sys.argv = ['distort.py', file_path, object_name, fouling_rating, method]
```

Run the script, replacing its path with the one on your machine:
```
exec(open('/path/to/distort.py').read());
```


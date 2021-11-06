#!/usr/bin/env python3

# In Blender, perform mesh modifications.
#
# Usage:
#   This script is to be run from within the Blender GUI. Tested in Blender 2.9.2.
#   distort.py <file_path> <object_prefix> [fouling_rating] [method]
#
#   Example:
#   From the console panel, run
#   >>> file_path = '/path/to/file.dae'
#   >>> object_prefix = 'Cube'
#   >>> fouling_rating = 10  # 0 to 100
#   >>> method = 'deform'  # method of distortion
#   >>> sys.argv = ['distort.py', file_path, object_prefix, fouling_rating, method]
#   >>> exec(open('/path/to/distort.py').read());
#

import bpy

import os
import sys


def find_target_object(object_prefix):

    target_obj = None
    object_name = None

    # Find the object that matches the name prefix. Assume the first one.
    for obj in bpy.data.objects:
        if obj.name.startswith(object_prefix):
            target_obj = bpy.data.objects[obj.name]
            object_name = obj.name
            break
    if target_obj == None:
        print('ERROR: Object with prefix [{}] not found'.format(object_prefix))
        return

    return target_obj, object_name


# shading: 'WIREFRAME', 'SOLID', 'MATERIAL', or 'RENDERED'
def viewport_shading(shading):
    print('Changing viewport shading to {}...'.format(shading))
    areas = bpy.context.workspace.screens[0].areas
    for ar in areas:
        for spc in ar.spaces:
            if spc.type == 'VIEW_3D':
                spc.shading.type = shading


def subdivision_modifier(obj, levels=2):

    if levels == 0:
        print('Subdivision modifier level is 0, skipping')
        return

    print('Applying subdivision modifier with level {}...'.format(levels))

    viewport_shading('WIREFRAME')

    # Select the object
    obj.select_set(True)
    # Set active object, otherwise incorrect context
    bpy.context.view_layer.objects.active = obj

    # subdivision modifier
    mod = obj.modifiers.new(name='Subdivision', type='SUBSURF')

    mod.subdivision_type = 'SIMPLE'
    mod.show_only_control_edges = False
    mod.levels = levels

    print(bpy.ops.object.modifier_apply(modifier='Subdivision'))


# Randomize mesh vertices
# offset (float in [-inf, inf], (optional)): Amount, Distance to offset. Meters
# uniform (float in [0, 1], (optional)): Uniform, Increase for uniform offset
#     distance
# normal (float in [0, 1], (optional)): Normal, Align offset direction to
#     normals
# seed (int in [0, 10000], (optional)): Random Seed
def mesh_vert_randomize(obj, offset=0.0, uniform=0.0, normal=1.0, seed=0):

    print('Applying mesh vertex randomization with offset {}...'.format(offset))

    # Set active object, otherwise incorrect context
    bpy.context.view_layer.objects.active = obj

    # Go into Edit mode
    bpy.ops.object.mode_set(mode='EDIT')

    bpy.ops.mesh.select_all(action='SELECT')

    # Offset is in meters
    print(bpy.ops.transform.vertex_random(offset=offset, uniform=uniform,
        normal=normal, seed=seed))

    # Go back to object mode
    bpy.ops.object.mode_set(mode='OBJECT')


def edge_subdivide(obj, ncuts=1, smooth=1.0):

    print('Applying edge subdivide with number of cuts {}...'.format(ncuts))

    # Set active object, otherwise incorrect context
    bpy.context.view_layer.objects.active = obj

    # Go into Edit mode
    bpy.ops.object.mode_set(mode='EDIT')

    bpy.ops.mesh.select_all(action='SELECT')

    print(bpy.ops.mesh.subdivide(number_cuts=ncuts, smoothness=smooth,
        ngon=True, quadcorner='STRAIGHT_CUT', fractal=0.0,
        fractal_along_normal=0.0, seed=0))

    # Go back to object mode
    bpy.ops.object.mode_set(mode='OBJECT')


def fill_holes(object_prefix):
    # TODO
    return


# rating: Fouling rating, in range [0, 100]
def distort(file_path, object_prefix, rating, method):

    # Clear scene
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

    # Open file
    if file_path.lower().endswith('dae'):
        bpy.ops.wm.collada_import(filepath=file_path)
    elif file_path.lower().endswith('obj'):
        bpy.ops.import_scene.obj(filepath=file_path, axis_forward='X',
            axis_up='Z')
    # TODO add imports for other formats. Trivial one-liners, but OBJ and
    # COLLADA are the most common used for Gazebo, others not needed now.
    else:
        print('ERROR: Only COLLADA (.dae) and OBJ formats are supported at the moment.')
        return

    target_obj, object_name = find_target_object(object_prefix)

    print('Distorting mesh [{}] for Fouling Rating {}...'.format(object_name,
        rating))

    # Normalize fouling rating
    RATING_MIN = 0
    RATING_MAX = 100
    rating_norm = (rating - RATING_MIN) / (RATING_MAX - RATING_MIN)

    METHODS = ['subdiv_mod', 'vert_rand', 'edge_subdiv']
    for step in method:

        if not step in METHODS:
            print('ERROR: Unrecognized step {}'.format(step))
            print('Available steps are:')
            for m in METHODS:
                print(m)
            print('Aborting')
            break

        if step == 'subdiv_mod':
            # Figure out levels magnitude
            SUBDIV_LVL_MIN = 0
            SUBDIV_LVL_MAX = 4
            # Must be integer
            subdiv_lvl = round(SUBDIV_LVL_MIN + (
                (SUBDIV_LVL_MAX - SUBDIV_LVL_MIN) * rating_norm))

            subdivision_modifier(target_obj, subdiv_lvl)

        elif step == 'vert_rand':
            # Meters
            VERT_RAND_MIN = 0
            VERT_RAND_MAX = 0.02
            # Figure out offset magnitude
            vert_rand_amt = VERT_RAND_MIN + (
                (VERT_RAND_MAX - VERT_RAND_MIN) * rating_norm)

            mesh_vert_randomize(target_obj, vert_rand_amt, uniform=0.0,
                normal=1.0, seed=0)

        elif step == 'edge_subdiv':

            edge_subdivide(target_obj)

    # Export result to file
    out_path = os.path.splitext(file_path)[0] + '_distort' + \
        os.path.splitext(file_path)[1]
    # TODO: COLLADA is not exporting texture correctly, not sure why
    if out_path.lower().endswith('dae'):
        bpy.ops.wm.collada_export(filepath=out_path)
    elif out_path.lower().endswith('obj'):
        bpy.ops.export_scene.obj(filepath=out_path, axis_forward='X',
            axis_up='Z')
    print('Exported result to [{}]'.format(out_path))


if __name__ == '__main__':
    # Default values
    fouling_rating = 10
    method = ['subdiv_mod', 'vert_rand', 'edge_subdiv']
    
    # Parse args
    if len(sys.argv) < 2:
        print('ERROR: Mesh name not provided. No object to distort.')
    else:
        file_path = sys.argv[1]
        object_prefix = sys.argv[2]
        if len(sys.argv) > 2:
            fouling_rating = int(sys.argv[3])
        if len(sys.argv) > 3:
            method = sys.argv[4]

        distort(file_path, object_prefix, fouling_rating, method)

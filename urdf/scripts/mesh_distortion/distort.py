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

import sys


# shading: 'WIREFRAME', 'SOLID', 'MATERIAL', or 'RENDERED'
def viewport_shading(shading):
    print('Changing viewport shading to {}...'.format(shading))
    areas = bpy.context.workspace.screens[0].areas
    for ar in areas:
        for spc in ar.spaces:
            if spc.type == 'VIEW_3D':
                spc.shading.type = shading


def subdivision_modifier(obj, levels=2):

    print('Applying subdivision modifier with level {}...'.format(levels))

    viewport_shading('WIREFRAME')

    # Select the object
    obj.select_set(True)

    # subdivision modifier
    mod = obj.modifiers.new(name='Subdivision', type='SUBSURF')

    mod.subdivision_type = 'SIMPLE'
    mod.show_only_control_edges = False
    mod.levels = levels

    # Set active object, otherwise modifier_apply() does not have context
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.modifier_apply(modifier='Subdivision')


# Randomize mesh vertices
# offset (float in [-inf, inf], (optional)): Amount, Distance to offset. Meters
# uniform (float in [0, 1], (optional)): Uniform, Increase for uniform offset
#     distance
# normal (float in [0, 1], (optional)): Normal, Align offset direction to
#     normals
# seed (int in [0, 10000], (optional)): Random Seed
def mesh_vert_randomize(obj, offset=0.0, uniform=0.0, normal=0.0, seed=0):

    # Go into Edit mode
    bpy.ops.object.mode_set(mode='EDIT')

    # Offset is in meters
    bpy.ops.transform.vertex_random(offset=offset, uniform=uniform,
        normal=normal, seed=seed)

    # Go back to object mode
    bpy.ops.object.mode_set(mode='OBJECT')


# rating: Fouling rating, in range [0, 100]
def deform(object_prefix, rating):

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

    print('Distorting mesh [{}] for Fouling Rating {}...'.format(object_name,
        rating))

    RATING_MIN = 0
    RATING_MAX = 100
    # Normalize
    rating_norm = (rating - RATING_MIN) / (RATING_MAX - RATING_MIN)

    SUBDIV_LVL_MIN = 0
    SUBDIV_LVL_MAX = 4
    # Must be integer
    subdiv_lvl = round(SUBDIV_LVL_MIN + (
        (SUBDIV_LVL_MAX - SUBDIV_LVL_MIN) * rating_norm))
    subdivision_modifier(target_obj, subdiv_lvl)

    # Meters
    VERT_RAND_MIN = 0
    VERT_RAND_MAX = 0.02
    vert_rand_amt = VERT_RAND_MIN + (
        (VERT_RAND_MAX - VERT_RAND_MIN) * rating_norm)
    mesh_vert_randomize(target_obj, vert_rand_amt, uniform=1.0, normal=1.0,
        seed=0)

    return object_name


def fill_holes(object_prefix):
    # TODO
    return



def distort(file_path, object_prefix, rating, method):

    methods = ['deform']
    if not method in methods:
        print('ERROR: Unrecognized distortion method {}'.format(method))
        print('Available methods are:')
        for m in methods:
            print(m)
        return

    # Clear scene
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

    # Open file
    bpy.ops.import_scene.obj(filepath=file_path, axis_forward='X', axis_up='Z')

    if method == 'deform':
        object_name = deform(object_prefix, fouling_rating)
        fill_holes(object_name)


if __name__ == '__main__':
    # Default values
    fouling_rating = 10
    method = 'deform'
    
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

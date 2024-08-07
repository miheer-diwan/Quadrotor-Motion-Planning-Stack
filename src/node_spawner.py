import bpy
import mathutils  # Import the mathutils module

# Delete all objects in the scene
#bpy.ops.object.select_all(action='DESELECT')
#bpy.ops.object.select_by_type(type='MESH')
#bpy.ops.object.delete()
bpy.ops.object.delete(use_global=False, confirm=False)

# Define a list of center locations in x, y, z format
center_locations = [
    (1.0, 2.0, 3.0),
    (4.0, 5.0, 6.0),
    (7.0, 8.0, 9.0),
    (10.0, 11.0, 12.0),  # Add more entries for additional spheres
    (13.0, 14.0, 15.0),  # Add more entries as needed
]

# Define the radius of the spheres
radius = 1.0

# Create spheres at the specified locations
spheres = []
for location in center_locations:
    bpy.ops.mesh.primitive_uv_sphere_add(radius=radius, location=location)
    spheres.append(bpy.context.active_object)

# Create cylinders between spheres
for i in range(len(spheres)-1):

    sphere1 = spheres[i]
    sphere2 = spheres[i+1]
    
    # Calculate the center of the two spheres
    start_location = sphere1.location
    print('start_location =',start_location)
    end_location = sphere2.location
    print('end_location =',end_location)

    
    # Calculate the length of the cylinder
    cylinder_length = (end_location - start_location).length
    
    # Calculate the rotation matrix for the cylinder
    direction = end_location - start_location
    up = mathutils.Vector((0, 0, 1))  # You can adjust the up direction as needed
    rotation_matrix = direction.to_track_quat('Z', 'Y').to_matrix().to_4x4()
    
    # Calculate the location of the cylinder (midpoint between spheres)
    cylinder_location = (start_location + end_location) / 2
    
    # Create the cylinder
    bpy.ops.mesh.primitive_cylinder_add(
        radius=radius / 2,  # Adjust the radius as needed
        depth=cylinder_length,
        location=cylinder_location,
    )
    cylinder = bpy.context.active_object
    cylinder.matrix_world = rotation_matrix

# Select the cylinders for easy manipulation in the 3D view
# bpy.ops.object.select_all(action='DESELECT')
# for obj in bpy.data.objects:
#     if obj.type == 'MESH':
#         obj.select_set(True)

# bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]

import bpy
import csv
import mathutils
from mathutils import Vector, Matrix

obj = bpy.context.active_object
if obj.mode != 'EDIT':
    bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.object.mode_set(mode='OBJECT')  # This updates the selection

# Get selected vertices
selected_verts = [v for v in obj.data.vertices if v.select]

# Export to CSV with full transformation matrices
with open('/home/marttave/projects/test/pre_generated_vertices/vertices.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    # Header for the 4x4 transformation matrix elements
    writer.writerow(['Index', 'M00', 'M01', 'M02', 'M03',
                     'M10', 'M11', 'M12', 'M13',
                     'M20', 'M21', 'M22', 'M23',
                     'M30', 'M31', 'M32', 'M33'])
    
    for v in selected_verts:
        # Get world space coordinates
        world_co = obj.matrix_world @ v.co
        
        # Get world space normal but invert it so it points inward
        normal = -v.normal  # Invert the normal here
        world_normal = (obj.matrix_world.to_3x3().inverted().transposed() @ normal).normalized()
        
        # Create a coordinate system (pose) at the vertex
        # Use the inverted normal as the Z axis (now pointing inward)
        z_axis = world_normal
        
        # Create X and Y axes orthogonal to the normal
        # We need a reference vector that's not parallel to z_axis
        # to compute the cross product
        ref = Vector((1.0, 0.0, 0.0))
        if abs(z_axis.dot(ref)) > 0.99:
            # If z_axis is too close to the X axis, use Y axis as reference
            ref = Vector((0.0, 1.0, 0.0))
        
        # X axis is perpendicular to both the normal and the reference
        x_axis = z_axis.cross(ref).normalized()
        
        # Y axis completes the right-handed system
        y_axis = z_axis.cross(x_axis).normalized()
        
        # Create a 4x4 transformation matrix
        pose_matrix = Matrix((
            (x_axis.x, y_axis.x, z_axis.x, world_co.x),
            (x_axis.y, y_axis.y, z_axis.y, world_co.y),
            (x_axis.z, y_axis.z, z_axis.z, world_co.z),
            (0.0, 0.0, 0.0, 1.0)
        ))
        
        # Flatten the matrix for CSV export
        matrix_values = [v.index] + [e for row in pose_matrix for e in row]
        writer.writerow(matrix_values)

print(f"Exported {len(selected_verts)} vertices with full pose matrices (z-axis pointing inward)")
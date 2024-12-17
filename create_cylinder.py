import gmsh
import math

# Initialize gmsh
gmsh.initialize()

# Create a new model
gmsh.model.add("cylinder")

# Define the cylinder parameters
radius = 3
height = 50
num_points = 100

# # Create points for the base circle
# points = []
# for i in range(num_points):
#     angle = 2 * 3.141592653589793 * i / num_points
#     x = radius * math.cos(angle)
#     y = radius * math.sin(angle)
#     points.append(gmsh.model.geo.addPoint(x, y, 0))

# # Create lines for the base circle
# lines = []
# for i in range(num_points):
#     lines.append(gmsh.model.geo.addLine(points[i], points[(i + 1) % num_points]))

# # Create a curve loop and surface for the base circle
# curve_loop = gmsh.model.geo.addCurveLoop(lines)
# surface = gmsh.model.geo.addPlaneSurface([curve_loop])

gmsh.model.occ.addCylinder(0, 0, 0,
                           0, 0, height,
                           radius)

# Synchronize the model
gmsh.model.occ.synchronize()

gmsh.option.setNumber("Mesh.MeshSizeFactor", 0.5)

# Generate the 2D mesh
gmsh.model.mesh.generate(2)

# Open the gmsh GUI to visualize the model
# gmsh.fltk.run()

# Save the mesh to a file
gmsh.write("cylinder.stl")

# Finalize gmsh
gmsh.finalize()
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
 
# Define the base radius and total height (along the z-axis) of the structure
base_radius = 10  # Base radius of the cone at x=0
total_height = 2 * base_radius  # Total height is twice the base radius
 
# The total length along the x-axis should be four times the total height
total_length = 4 * total_height
 
def generate_distributed_cubes(total_length, base_radius, num_cubes):
    cubes = []
    half_length = total_length / 2
    volume_per_cube = (np.pi * base_radius**2 * half_length) / num_cubes
 
    while len(cubes) < num_cubes:
        # Randomly choose a position along the x-axis
        x_position = np.random.uniform(0, half_length)
        scale = (half_length - x_position) / half_length
        current_radius = base_radius * scale
 
        # Calculate the size of the cube based on the remaining volume to be filled
        remaining_cubes = num_cubes - len(cubes)
        cube_volume = volume_per_cube * remaining_cubes
        cube_size = np.cbrt(cube_volume / np.pi) * scale
 
        if cube_size > 2 * current_radius:
            cube_size = 2 * current_radius
 
        # Randomly position the cube within the current cross-section
        angle = np.random.uniform(0, 2 * np.pi)
        radius = np.random.uniform(0, current_radius - cube_size / 2)
        y = radius * np.cos(angle)
        z = radius * np.sin(angle)
 
        cubes.append({'position': (x_position, y, z), 'size': cube_size})
        cubes.append({'position': (-x_position, y, z), 'size': cube_size})  # Mirror on the negative x-axis
 
        if len(cubes) > num_cubes:
            break
 
    return cubes[:num_cubes]  # Ensure we return exactly num_cubes
 
# Total number of cubes to distribute
num_cubes = 100  # Total number, not per side
 
# Generate the cubes distributed throughout the cone
distributed_cubes = generate_distributed_cubes(total_length, base_radius, num_cubes)
 
# Plotting for visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
colors = plt.cm.viridis(np.linspace(0, 1, len(distributed_cubes)))
 
# Plotting the cubes
for cube, color in zip(distributed_cubes, colors):
    x, y, z = cube['position']
    size = cube['size']
    ax.bar3d(x, y, z, size, size, size, color=color)
 
# Set labels and title
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')
ax.set_title('Distributed Cubes Forming a Cone Shape Along X-Axis')
 
# Set the aspect of the plot to match the 4:1 ratio along the x and z axes
ax.set_box_aspect((total_length, total_height, total_height))  # Aspect ratio 4:1 for x:z
 
# Show the plot
plt.show()
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# Define the parameters for the elliptical paraboloid
a = 1  # Semi-major axis
b = 1  # Semi-minor axis
h = 2  # Height scaling factor

# Generate the meshgrid for the surface
u = np.linspace(-2, 2, 100)
v = np.linspace(-2, 2, 100)
u, v = np.meshgrid(u, v)

# Calculate the x, y, and z coordinates of the surface
x = a * u
y = b * v
z = (x**2 / a**2 + y**2 / b**2) * h

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the surface
ax.plot_surface(x, y, z, cmap='viridis')

# Calculate the cross-section at x=1
# x_cross = 0
# y_cross = np.linspace(-2, 2, 10000)
# z_cross = (x_cross**2 / a**2 + y_cross**2 / b**2) * h

# # Plot the cross-section curve
# ax.plot(x_cross * np.ones_like(y_cross), y_cross, z_cross-0.5, color='r', linewidth=3)

# x_cross = np.linspace(-2, 2, 10000)
# y_cross = 0
# z_cross = (x_cross**2 / a**2 + y_cross**2 / b**2) * h
# ax.plot(x_cross, y_cross* np.ones_like(x_cross), z_cross-0.5, color='r', linewidth=3)


# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

plt.show()
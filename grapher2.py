import numpy as np
from scipy.interpolate import CubicSpline, interp1d
from scipy.integrate import cumtrapz
import matplotlib.pyplot as plt

moveList = [[0,2,2], # actual coords in grapher.py file
            [1.6,10.8,2],
            [8.9,18.5,2],
            [18.7,22.2,2],
            # [30,22,2,2],
            # [39,19.2,2],
            [45.7,11.7,2],
            [45.7,2.2,2],
            [40.3,-4.8,2],
            [30.7,-7.9,2],
            [18.5,-7.9,2],
            [9.5,-5.1,2],  
            [0,-2,3.2]]

waypoints = []
for i in moveList:
    waypoints.append([i[0],i[1],i[2]])

waypoints = np.array(waypoints)
t = np.linspace(0, 1, len(waypoints))

spline_x = CubicSpline(t, waypoints[:, 0])
spline_y = CubicSpline(t, waypoints[:, 1])
spline_z = CubicSpline(t, waypoints[:, 2])

num_points = 50

# t_new = np.linspace(0, 1, num_points)

# path_x = spline_x(t_new)
# path_y = spline_y(t_new)
# path_z = spline_z(t_new)
# path = np.stack((path_x, path_y, path_z), axis=-1)

dt = 0.01  # Small time step for numerical differentiation
t_fine = np.arange(0, 1, dt)
dx_dt = spline_x(t_fine, 1)
dy_dt = spline_y(t_fine, 1)
dz_dt = spline_z(t_fine, 1)

# Compute the differential arc length
d_length = np.sqrt(dx_dt**2 + dy_dt**2 + dz_dt**2)
arc_length = cumtrapz(d_length, t_fine, initial=0)

# Step 2: Interpolate to find evenly spaced points along arc length
total_length = arc_length[-1]
even_arc_lengths = np.linspace(0, total_length, num_points)
t_even = interp1d(arc_length, t_fine)(even_arc_lengths)

# Step 3: Evaluate splines at these new t values
path_x = spline_x(t_even)
path_y = spline_y(t_even)
path_z = spline_z(t_even)
path = np.stack((path_x, path_y, path_z), axis=-1)

for i in waypoints:
    plt.plot(i[0], i[1], marker='o', color='y', markersize=20)

for i in path:
    plt.plot(i[0], i[1], marker='o', color='g', markersize=10) 

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Uneven Spacing')
plt.grid(True)

plt.show()
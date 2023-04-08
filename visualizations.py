import numpy as np
import matplotlib.pyplot as plt
import pdb

binary_occupancy_grid = np.load("binary_occupancy_grid.npy")
waypoints_log = np.load("waypoints_log.npy")
goal_grid_index = np.load("goal_grid_index.npy")

plt.imshow(binary_occupancy_grid, origin="lower", cmap="Greys")
plt.title("Binary Occupancy Grid")
plt.xlabel("Eastings")
plt.ylabel("Northings")

print(waypoints_log)
grid_eastings = waypoints_log[:, 0]
grid_northings = waypoints_log[:, 1]

plt.plot(grid_eastings, grid_northings)
plt.scatter(goal_grid_index[0], goal_grid_index[1])

plt.show()

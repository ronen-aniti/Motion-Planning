import numpy as np
import matplotlib.pyplot as plt
import pdb

binary_occupancy_grid = np.load("binary_occupancy_grid.npy")
grid_history = np.load("grid_history.npy")

plt.imshow(binary_occupancy_grid, origin="lower", cmap="Greys")
plt.title("Binary Occupancy Grid")
plt.xlabel("Eastings")
plt.ylabel("Northings")


grid_eastings = grid_history[:, 1]
grid_northings = grid_history[:, 0]
timestamps = grid_history[:, 2]

print(grid_eastings)
print(grid_northings)
print(grid_history)
plt.scatter(grid_eastings, grid_northings, c=timestamps, cmap="viridis")

plt.show()

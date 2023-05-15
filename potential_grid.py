class PotentialGrid(Planner):
	# High potential means very close to danger
	# Low potential means very safe or very close to goal
	# Distance transform from nearest obstacle
	# Square the distances
	# Multiply those values by an obstacle gain
	# Save that grid
	# Compute a grid using a distance transform from the goal state
	# Square those values
	# Multiply those values by a negative gain
	# Compute the gradient of the 
import matplotlib.cm as cm
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np
import pdb
from sklearn.neighbors import KDTree
from typing import List

class State:
	def __init__(self, position: np.ndarray, heading: float, waypoint: np.ndarray, potential: float, gradient: np.ndarray):
		self.position = position
		self.heading = heading
		self.waypoint = waypoint
		self.potential = potential
		self.gradient = gradient

class Bounds: 
	def __init__(self, minimum: float, maximum: float):
		self.minimum = minimum
		self.maximum = maximum

class Obstacle:
	def __init__(self, position: np.ndarray, safety: float):
		self.position = position
		self.safety = safety

class Environment:
	def __init__(self):
		self.x_bounds = None
		self.y_bounds = None
		self.start = None
		self.goal = None
		self.obstacle_list = []
		self.obstacle_kdtree = None

	def set_x_bounds(self, minimum: float, maximum: float):
		self.x_bounds = Bounds(minimum, maximum)

	def set_y_bounds(self, minimum: float, maximum: float):
		self.y_bounds = Bounds(minimum, maximum)

	def set_start(self, x: float, y: float):
		self.start = np.array([x, y])	

	def set_goal(self, x: float, y: float):
		self.goal = np.array([x, y])

	def add_obstacles_as_list(self, obstacle_list: List[Obstacle]):
		self.obstacle_list.extend(obstacle_list)
		self.obstacle_kdtree = self.build_obstacle_kdtree() # Rebuild tree
		self.build_obstacle_kdtree()

	def build_obstacle_kdtree(self):
		positions = []
		for obstacle in self.obstacle_list:
			positions.append([obstacle.position[0], obstacle.position[1]])
		positions = np.array(positions)
		self.obstacle_kdtree = KDTree(positions)

class ObstacleGenerator:
	def __init__(self, environment: Environment):
		self.environment = environment
		
	def generate_n_obstacles(self, num_obstacles_to_add: int):
		obstacle_list = []
		for _ in range(num_obstacles_to_add):
			x = np.random.uniform(low=self.environment.x_bounds.minimum, high=self.environment.x_bounds.maximum)
			y = np.random.uniform(low=self.environment.y_bounds.minimum, high=self.environment.y_bounds.maximum)
			position = np.array([x, y])
			safety = np.sqrt(2) * 5
			new_obstacle = Obstacle(position, safety)
			obstacle_list.append(new_obstacle)
		return obstacle_list

class StateGenerator:
	def __init__(self, environment: Environment, resolution: float):
		self.environment = environment
		self.resolution = resolution
		
	def generate_states(self):
		states_list = []
		x_values = np.arange(environment.x_bounds.minimum, environment.x_bounds.maximum + resolution, resolution)
		y_values = np.arange(environment.y_bounds.minimum, environment.y_bounds.maximum + resolution, resolution)
		for x in x_values:
			for y in y_values:
				position = np.array([x, y])
				heading = 0
				waypoint = np.array([x, y, heading])
				potential = self.compute_potential(x, y)
				gradient = self.compute_gradient(x, y)
				states_list.append(State(position, heading, waypoint, potential, gradient))

		return states_list

	def compute_gradient(self, x: float, y: float) -> np.ndarray:
		delta = self.resolution / 2

		x_plus = x + delta
		x_minus = x - delta
		y_plus = y + delta
		y_minus = y - delta

		potential_x_plus = self.compute_potential(x_plus, y)
		potential_x_minus = self.compute_potential(x_minus, y)
		potential_y_plus = self.compute_potential(x, y_plus)
		potential_y_minus = self.compute_potential(x, y_minus)

		gradient_x = (potential_x_plus - potential_x_minus) / (2 * delta)
		gradient_y = (potential_y_plus - potential_y_minus) / (2 * delta)

		return np.array([gradient_x, gradient_y])

	def compute_potential(self, x: float, y: float) -> float:
		ko = 10
		kg = 10000
		do = self.compute_distance_to_nearest_obstacle(x, y)
		dg = self.compute_distance_to_goal(x, y)
		potential = ko/do**2 - kg/dg**2
		return potential

	def compute_distance_to_nearest_obstacle(self, x, y) -> float:
		position = np.array([x, y])
		nearest_obstacle_index = self.environment.obstacle_kdtree.query([position])[1][0][0]
		nearest_obstacle = self.environment.obstacle_list[nearest_obstacle_index]
		obstacle_position = nearest_obstacle.position
		return np.linalg.norm(position - obstacle_position)

	def compute_distance_to_goal(self, x, y) -> float:
		position = np.array([x, y])
		return np.linalg.norm(position - environment.goal) 


# Main
environment = Environment()
environment.set_x_bounds(-100, 100)
environment.set_y_bounds(-100, 100)
environment.set_start(-50, 50)
environment.set_goal(90, -90)
obstacle_generator = ObstacleGenerator(environment)
obstacle_list = obstacle_generator.generate_n_obstacles(25)
environment.add_obstacles_as_list(obstacle_list)
resolution = 1
state_generator = StateGenerator(environment, resolution)
states_list = state_generator.generate_states()


scaling_factor = 100
current = environment.start
trajectory = []
for _ in range(100):
	#if np.linalg.norm(environment.goal - current) <= 1:
	#	print("break")
	#	break
	gradient = state_generator.compute_gradient(current[0], current[1])
	print("gradient", gradient)
	new = current + scaling_factor * -1 * gradient
	print(new)
	trajectory.append(new)
	current = new

trajectory = np.array(trajectory)







print("Finished with calculations")

positions = [state.position for state in states_list]
gradients = [state.gradient for state in states_list]
positions = np.array(positions)
gradients = np.array(gradients)
gradient_magnitudes = np.linalg.norm(gradients, axis=1)
normalized_gradients = gradients / gradient_magnitudes[:, np.newaxis]
colors = cm.viridis(gradient_magnitudes / np.max(gradient_magnitudes))
plt.quiver(positions[:, 0], positions[:, 1], -normalized_gradients[:, 0], -normalized_gradients[:, 1], color=colors, scale=150)
plt.plot(environment.start[0], environment.start[1], "go", label="Start")
plt.plot(environment.goal[0], environment.goal[1], "ro", label="Goal")
plt.plot(trajectory[:, 0], trajectory[:, 1], 'r--', label='Trajectory')
for obstacle in environment.obstacle_list:
	plt.plot(obstacle.position[0], obstacle.position[1], "ko")

plt.legend()
plt.grid()
plt.axis("equal")
plt.title("Artificial Potential Field")
plt.show()
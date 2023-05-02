import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np
import pdb
from sklearn.neighbors import KDTree
from typing import List

class Bounds:
	def __init__(self, minimum: float, maximum:float):
		self.minimum = minimum
		self.maximum = maximum

class Obstacle:
	def __init__(self, xy_position: np.ndarray, safety: float):
		self.xy_position = xy_position
		self.safety = safety

class Environment:
	def __init__(self, x_bounds: Bounds, y_bounds: Bounds, number_of_obstacles: int, start_position: np.ndarray, goal_position: np.ndarray, current_position: np.ndarray):
		self.current_position = current_position
		self.x_bounds = x_bounds
		self.y_bounds = y_bounds
		self.start_position = start_position
		self.goal_position = goal_position
		self.obstacle_list = self.generate_n_obstacles(number_of_obstacles)
		self.obstacle_xy_positions = self.determine_obstacle_xy_positions()
		self.obstacle_kdtree = self.build_kd_tree(self.obstacle_list)


	def generate_n_obstacles(self, number_of_obstacles) -> List[Obstacle]:
		HALFSIZE = 5
		SAFETY_RADIUS = HALFSIZE * np.sqrt(2)

		obstacle_list = []
		obstacle_xy_positions = []
		for i in range(number_of_obstacles):
			x_position = np.random.uniform(low=self.x_bounds.minimum, high=self.x_bounds.maximum)
			y_position = np.random.uniform(low=self.y_bounds.minimum, high=self.y_bounds.maximum)
			
			xy_position = np.array([x_position, y_position])
			obstacle_xy_positions.append(xy_position)

			obstacle_object = Obstacle(xy_position, SAFETY_RADIUS)
			obstacle_list.append(obstacle_object)

		return obstacle_list

	def determine_obstacle_xy_positions(self) -> np.ndarray:
		obstacle_xy_positions = [obstacle.xy_position for obstacle in self.obstacle_list]
		obstacle_xy_positions = np.vstack(obstacle_xy_positions)
		return obstacle_xy_positions

	def build_kd_tree(self, obstacle_list) -> KDTree:
		
		obstacle_xy_positions = []
		for obstacle in obstacle_list:
			obstacle_xy_positions.append(obstacle.xy_position)

		obstacle_xy_positions = np.vstack(obstacle_xy_positions)

		obstacle_kdtree = KDTree(obstacle_xy_positions) # references the indices of obstacle_list

		return obstacle_kdtree

	def generate_start_or_goal_position(self):
		valid_position = False
		while not valid_position:
			x_position = np.random.uniform(low=self.x_bounds.minimum, high=self.x_bounds.maximum)
			y_position = np.random.uniform(low=self.y_bounds.minimum, high=self.y_bounds.maximum)
			xy_position = np.array([x_position, y_position])
			nearby_obstacle_indices = self.obstacle_kdtree.query_radius([xy_position], self.obstacle_list[0].safety)[0]
			if len(nearby_obstacle_indices) == 0:
				valid_position = True
		if valid_position:
			return xy_position



## Simulate trajectory
def simulate(environment: Environment, current_position: np.ndarray, step_size: float) -> np.ndarray:
	
	# Gains
	ks = 100
	ko = 10
	kg = 1000
	
	# Determine the position of the nearest obstacle
	nearest_obstacle_index = environment.obstacle_kdtree.query([current_position])[1][0][0]
	nearest_obstacle = environment.obstacle_list[nearest_obstacle_index]
	obstacle_position = nearest_obstacle.xy_position

	# Establish the start and goal positions of the potential field
	start_position = environment.start_position
	goal_position = environment.goal_position

	# Calculate the attractive and repulsive forces "felt" at the current position
	vector_from_start_to_current = current_position - start_position
	distance_start_to_current = np.linalg.norm(vector_from_start_to_current)
	unit_vector_start_to_current = vector_from_start_to_current / distance_start_to_current
	force_start_to_current = ks / distance_start_to_current**2 * unit_vector_start_to_current

	vector_current_to_goal = goal_position - current_position
	distance_current_to_goal = np.linalg.norm(vector_current_to_goal)
	unit_vector_current_to_goal = vector_current_to_goal / distance_current_to_goal
	force_current_to_goal = kg / distance_current_to_goal**2 * unit_vector_current_to_goal

	vector_from_obstacle_to_current = current_position - obstacle_position
	distance_from_obstacle_to_current = np.linalg.norm(vector_from_obstacle_to_current)
	unit_vector_from_obstacle_to_current = vector_from_obstacle_to_current / distance_from_obstacle_to_current
	force_obstacle_to_current = ko / distance_from_obstacle_to_current**2 * unit_vector_from_obstacle_to_current

	force_resultant_at_current = force_start_to_current + force_obstacle_to_current + force_current_to_goal
	force_mag_at_current = np.linalg.norm(force_resultant_at_current)
	unit_vector_at_current = force_resultant_at_current / force_mag_at_current
	new_position = current_position + step_size * unit_vector_at_current

	return new_position

class PotentialField:
	def __init__(self, environment, resolution: float):
		self.environment = environment
		self.x_positions = np.arange(environment.x_bounds.minimum, environment.x_bounds.maximum+resolution, resolution)
		self.y_positions = np.arange(environment.y_bounds.minimum, environment.y_bounds.maximum+resolution, resolution)
		self.vector_field = self.compute_field()
	
	def draw_trajectory_from(self, current_position: np.ndarray, step_size: float):

		trajectory = []
		for _ in range(10000):
			new_position = simulate(self.environment, current_position, step_size)
			trajectory.append(new_position)
			if np.linalg.norm(self.environment.goal_position - new_position) <= step_size:
				print("break")
				break
			current_position = new_position

		return trajectory

	def compute_field(self):
		vector_field = []
		# Gains
		ks = 0
		ko = 10
		kg = 1000 #1000

		for x_value in self.x_positions:
			for y_value in self.y_positions:

				current_position = np.array([x_value, y_value])
				# Determine the position of the nearest obstacle
				nearest_obstacle_index = self.environment.obstacle_kdtree.query([current_position])[1][0][0]
				nearest_obstacle = self.environment.obstacle_list[nearest_obstacle_index]
				obstacle_position = nearest_obstacle.xy_position

				# Establish the start and goal positions of the potential field
				start_position = self.environment.start_position
				goal_position = self.environment.goal_position

				# Calculate the attractive and repulsive forces "felt" at the current position
				vector_from_start_to_current = current_position - start_position
				distance_start_to_current = np.linalg.norm(vector_from_start_to_current)
				unit_vector_start_to_current = vector_from_start_to_current / distance_start_to_current
				force_start_to_current = ks / distance_start_to_current**2 * unit_vector_start_to_current

				vector_current_to_goal = goal_position - current_position
				distance_current_to_goal = np.linalg.norm(vector_current_to_goal)
				unit_vector_current_to_goal = vector_current_to_goal / distance_current_to_goal
				force_current_to_goal = kg / distance_current_to_goal**2 * unit_vector_current_to_goal

				vector_from_obstacle_to_current = current_position - obstacle_position
				distance_from_obstacle_to_current = np.linalg.norm(vector_from_obstacle_to_current)
				unit_vector_from_obstacle_to_current = vector_from_obstacle_to_current / distance_from_obstacle_to_current
				force_obstacle_to_current = ko / distance_from_obstacle_to_current**2 * unit_vector_from_obstacle_to_current

				force_resultant_at_current = force_start_to_current + force_obstacle_to_current + force_current_to_goal		
				force_mag = np.linalg.norm(force_resultant_at_current)
				x_force = np.dot(force_resultant_at_current, np.array([1, 0]))
				y_force = np.dot(force_resultant_at_current, np.array([0, 1]))
				vector_field.append([x_value, y_value, x_force, y_force, force_mag])

		return vector_field



# Main
x_bounds = Bounds(-50, 50)
y_bounds = Bounds(-50, 50)
number_of_obstacles = 50
start_position = np.array([np.random.uniform(low=x_bounds.minimum, high=x_bounds.maximum), np.random.uniform(low=y_bounds.minimum, high=y_bounds.maximum)])
goal_position = np.array([np.random.uniform(low=x_bounds.minimum, high=x_bounds.maximum), np.random.uniform(low=y_bounds.minimum, high=y_bounds.maximum)])
current_position = np.array([np.random.uniform(low=x_bounds.minimum, high=x_bounds.maximum), np.random.uniform(low=y_bounds.minimum, high=y_bounds.maximum)])
environment = Environment(x_bounds, y_bounds, number_of_obstacles, start_position, goal_position, current_position)
resolution = 1
potential_field = PotentialField(environment, resolution)
print("Done calculating")
fig, ax = plt.subplots()

X = [vector[0] for vector in potential_field.vector_field]
Y = [vector[1] for vector in potential_field.vector_field]
U = [vector[2]/vector[4] for vector in potential_field.vector_field]
V = [vector[3]/vector[4] for vector in potential_field.vector_field]
M = [vector[4] for vector in potential_field.vector_field]


ax.quiver(X, Y, U, V, alpha=0.5, scale=50, color=(0, 0.5, 0.5), headwidth=1, headlength=3)

ax.scatter(environment.obstacle_xy_positions[:, 0], environment.obstacle_xy_positions[:, 1], color="black", label="Obstacle")
ax.scatter(environment.start_position[0], environment.start_position[1], color="green", label="Start")
ax.scatter(environment.goal_position[0], environment.goal_position[1], color="red", label="Goal", alpha=1)
ax.scatter(environment.current_position[0], environment.current_position[1], color="purple", label="Current")


ax.set_xlim(x_bounds.minimum, x_bounds.maximum)
ax.set_ylim(y_bounds.minimum, y_bounds.maximum)
ax.set_title("Environment Represented Using an Artificial Force Field")
step_size = 0.1
trajectory_points = potential_field.draw_trajectory_from(current_position, step_size)
ax.plot(np.array(trajectory_points)[:,0], np.array(trajectory_points)[:,1], color=(0.5, 0, 0.5), label="Trajectory")
ax.legend()


plt.show()
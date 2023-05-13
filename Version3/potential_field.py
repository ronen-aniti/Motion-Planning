from bounds import Bounds
import copy
from environment import Environment
from local_position import LocalPosition
import matplotlib.pyplot as plt
import numpy as np
from start_goal_pair import StartGoalPair
from state import State
from state_collection import StateCollection
from typing import List
from planner import Planner

from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from matplotlib.patches import Circle
import matplotlib.patches as patches
import matplotlib.pyplot as plt

import pdb


class PotentialField(Planner):

	def __init__(self, environment, start_state, goal_state):
		self._environment = environment
		self._start_state = start_state
		self._goal_state = goal_state
		self._takeoff_altitude = self._goal_state.local_position.down
		self._resolution = 5 # meter cubed
		self._step_size = .1 # meter
		self._resolution_of_collision_detector = self._environment.obstacles.safety
		self._state_sequence = self._determine_path()
		self._shortened_path = self._shorten_path()
	
	def _return_nearest_obstacle_ground_position(self, current_position: np.ndarray) -> np.array:
		# Determine the position of the nearest obstacle
		nearest_obstacle_index = self._environment.obstacles.tree.query([current_position])[1][0][0]
		nearest_obstacle = self._environment.obstacles.list[nearest_obstacle_index]
		obstacle_position = np.array([nearest_obstacle.local_position.north,
									  nearest_obstacle.local_position.east])
		return obstacle_position

	def _compute_force_from_start_to_current(self, current_position: np.ndarray, start_position: np.ndarray) -> np.ndarray:
		ks = 0
		# Calculate the attractive and repulsive forces "felt" at the current position
		#vector_from_start_to_current = current_position - start_position
		#distance_start_to_current = np.linalg.norm(vector_from_start_to_current)
		#unit_vector_start_to_current = vector_from_start_to_current / distance_start_to_current
		force_start_to_current = np.array([0, 0]) #Ignore for now: ks / distance_start_to_current**2 * unit_vector_start_to_current
		return force_start_to_current

	def _compute_force_from_current_to_goal(self, current_position: np.ndarray, goal_position: np.ndarray) -> np.ndarray:
		kg = 1000
		vector_current_to_goal = goal_position - current_position
		distance_current_to_goal = np.linalg.norm(vector_current_to_goal)
		unit_vector_current_to_goal = vector_current_to_goal / distance_current_to_goal
		force_current_to_goal = kg / distance_current_to_goal**2 * unit_vector_current_to_goal
		return force_current_to_goal

	def _compute_force_from_obstacle_to_current(self, current_position: np.ndarray, obstacle_position: np.ndarray, force_current_to_goal: np.ndarray) -> np.ndarray:
		ko = 10
		vector_from_obstacle_to_current = current_position - obstacle_position
		distance_from_obstacle_to_current = np.linalg.norm(vector_from_obstacle_to_current)
		if distance_from_obstacle_to_current <= self._environment.obstacles.safety:
			if distance_from_obstacle_to_current == 0:
				unit_vector_from_obstacle_to_current = np.array([0, 0])
			else:
				unit_vector_from_obstacle_to_current = vector_from_obstacle_to_current / distance_from_obstacle_to_current
			force_obstacle_to_current = np.linalg.norm(force_current_to_goal) * unit_vector_from_obstacle_to_current # This line results in a strengthening of the obstacle field in the vacinity of the goal, reducing the chance of collisions with obstacles near the goal state.
		else:
			unit_vector_from_obstacle_to_current = vector_from_obstacle_to_current / distance_from_obstacle_to_current
			force_obstacle_to_current = ko / (distance_from_obstacle_to_current)**2 * unit_vector_from_obstacle_to_current
		return force_obstacle_to_current

	def _compute_force_resultant_at_current(self, current_position: np.ndarray, start_position: np.ndarray, goal_position: np.ndarray) -> np.ndarray:
		# Compute force resultant at the current state's position
		obstacle_ground_position = self._return_nearest_obstacle_ground_position(current_position)
		force_start_to_current = self._compute_force_from_start_to_current(current_position, start_position)
		force_current_to_goal = self._compute_force_from_current_to_goal(current_position, goal_position)
		force_obstacle_to_current = self._compute_force_from_obstacle_to_current(current_position, obstacle_ground_position, force_current_to_goal)
		force_resultant_at_current = force_start_to_current + force_obstacle_to_current + force_current_to_goal
		return force_resultant_at_current

	def _determine_resultant_force_unit_vector_at_current(self, current_state: State) -> np.ndarray:
		# Establish the start and goal positions of the potential field. Use some simplifying conditions for this prototype implementation. 
		goal_position = self._goal_state.ground_position
		start_position = self._start_state.ground_position
		current_position = current_state.ground_position

		force_resultant_at_current = self._compute_force_resultant_at_current(current_position, start_position, goal_position)

		# Step forward in the direction of that force, and establish a state at that new position
		force_mag_at_current = np.linalg.norm(force_resultant_at_current)
		unit_vector_at_current = force_resultant_at_current / force_mag_at_current

		return unit_vector_at_current

	def _compute_next_state(self, current_state: State) -> State:
		unit_vector_at_current = self._determine_resultant_force_unit_vector_at_current(current_state)
		current_position = current_state.ground_position
		new_position = current_position + self._step_size * unit_vector_at_current
		new_position = LocalPosition(new_position[0], new_position[1], self._takeoff_altitude)
		new_state = State(self._environment, self._goal_state.local_position, new_position, heading=0, parent_state=current_state)

		return new_state

	def _determine_path(self) -> List[State]:
		# Search the graph of potential field states for the minimum cost path. Or the path that is monotonically decreasing. 
		
		trajectory = []
		current_state = self._start_state
		
		for _ in range(10000):
			new_state = self._compute_next_state(current_state)
			trajectory.append(new_state)
			if np.linalg.norm(self._goal_state.position_in_3d - new_state.position_in_3d) <= self._step_size:
				return trajectory
			current_state = new_state
			#pdb.set_trace()
		

		return trajectory

	def _shorten_path(self) -> List[State]:
		"""
		Shorten the path to the goal by iteratively removing intermediate unnecessary intermediate states. 

		Returns: 
			A shortned list of 'State' objects representing the path to the goal. 
		"""

		# TODO: MAKE THIS MORE MODULAR 
		
		# Make a copy of the path of states from start to goal
		path = copy.deepcopy(self._state_sequence)

		
		for i in range(50): #Experiment with the number of times to shorten the path
			# Start from the last state in the path
			start = path[-1]

			# While the current state has a parent
			while start.parent_state is not None:
				if start.parent_state.parent_state is not None:

					# Take the grandparent as the subgoal
					subgoal = start.parent_state.parent_state

					# Compute the unit vector from the current state to the subgoal 
					vector = subgoal.position_in_3d - start.position_in_3d
					vector_mag = np.linalg.norm(vector)
					unit_vector = vector / vector_mag

					# Check for collisions along the vector between the start and subgoal states
					collides = False
					for step in np.arange(self._resolution_of_collision_detector, vector_mag, self._resolution_of_collision_detector):
						local_position = start.position_in_3d + step * unit_vector
						local_position = LocalPosition(local_position[0], local_position[1], local_position[2])
						state = State(self._environment, self._goal_state.local_position, local_position)
						collides = self._environment.state_collides_with_obstacle(state)
						if collides:
							start = start.parent_state
							break

					# If there's no collision, set the parent state of the current state to be the subgoal, and move to the subgoal
					if not collides:
						start.parent_state = subgoal
						start = subgoal

					# If the start state is reached, break out of the loop
					if start == self._start_state:
						break
				else:
					break

		# Generate the shortened path, and compute its cost, by traversing from the last state to the start state. 
		shortened_path = []
		cost = 0
		current = path[-1]
		while current.parent_state is not None:
			shortened_path.append(current)
			cost += np.linalg.norm(current.position_in_3d - current.parent_state.position_in_3d)
			current = current.parent_state
		shortened_path.append(self._start_state)
		shortned_path = shortened_path[::-1]
		print(f"The cost is {cost: .2f} m.")

		return shortned_path

	def return_waypoints(self):
		return [state.waypoint for state in self._shortened_path]

	def _create_a_mesh_for_visualization(self):
		mesh_states = []
		for north in np.arange(self._environment.north_bounds.minimum, self._environment.north_bounds.maximum, self._resolution):
			for east in np.arange(self._environment.east_bounds.minimum, self._environment.east_bounds.maximum, self._resolution):
				state = State(self._environment, self._goal_state.local_position, LocalPosition(north, east, self._takeoff_altitude))
				if not self._environment.state_collides_with_obstacle(state):
					unit_vector_at_current = self._determine_resultant_force_unit_vector_at_current(state)
					mesh_states.append((state.ground_position, unit_vector_at_current))

		return mesh_states

	def visualize(self, plot_original_trajectory=True, plot_with_gradient_field=True, plot_shortened_path=True):
		# Put all the potential states on the map, labeled with their potential values, having a colormap of their own to indicate their total potential.
		fig, ax = self._environment.visualize()

		if plot_with_gradient_field:
			mesh_states = self._create_a_mesh_for_visualization()
			X = [state[0][0] for state in mesh_states]
			Y = [state[0][1] for state in mesh_states]
			U = [state[1][0] for state in mesh_states]
			V = [state[1][1] for state in mesh_states]
			plt.quiver(X, Y, U, V, alpha=0.5)

		if plot_original_trajectory:
			potential_field_trajectory = [state.local_position for state in self._state_sequence]
			ax.plot([p.north for p in potential_field_trajectory], [p.east for p in potential_field_trajectory], label="Artificial force-field trajectory", color="blue", alpha=0.75)

		if plot_shortened_path:
			shortened_path = [state.local_position for state in self._shortened_path]
			ax.plot([p.north for p in shortened_path], [p.east for p in shortened_path], label="Shortned path", color="lime")
		plt.scatter(self._start_state.local_position.north, self._start_state.local_position.east, color='green', marker='o', label='Start state')
		plt.scatter(self._goal_state.local_position.north, self._goal_state.local_position.east, color='red', marker='o', label='Goal state') 
		ax.set_xlim(self._environment.north_bounds.minimum, self._environment.north_bounds.maximum)
		ax.set_ylim(self._environment.east_bounds.minimum, self._environment.east_bounds.maximum)
		plt.legend()
		plt.show()



	@property
	def state_sequence(self):
		return self._state_sequence

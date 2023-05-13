from environment import Environment
from state import State
from local_position import LocalPosition
from planner import Planner
from typing import List
from state_collection import StateCollection
import pdb
import numpy as np
import copy

from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from matplotlib.patches import Circle
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

class RapidlyExploringRandomTree(Planner):
	"""A variation of the rapidly-exploring random tree (RRT) pathfinding algorithm, this implmentation features a sampling routine that alternates between selecting a state at random and sampling the goal state."""
	def __init__(self, environment: Environment, start_state: State, goal_state: State):
		super().__init__(environment, start_state, goal_state)
		self._tree_altitude = goal_state.local_position.down 
		self._step_size_of_tree = self._determine_step_size_of_tree()
		self._resolution_of_collision_detector = self._determine_resolution_of_collision_detector()
		self._maximum_number_of_iterations = self._determine_maximum_number_of_iterations()
		self._takeoff_state =  None

		

	def run(self): 
		# Determine the takeoff state based on the target tree altitude
		self._takeoff_state = self._determine_takeoff_state(self._tree_altitude)
		
		# Initialize the list of sampled states with the start and takeoff states
		self._sample_states = [self._start_state, self._takeoff_state]

		# Sample additional states until a state near the goal is reached
		self._sample_until_near_goal()

		# Connect the goal state to the nearest sampled state in the tree
		self._state_space = self._connect_goal_state()			

		# Rewire the states in the tree to improve the path
		self._rewire_the_states()

		# Determine the path of states and its associated cost
		self._path, self._cost = self._determine_path_of_states()

		return [state.waypoint for state in self._path.list]

	def _determine_takeoff_state(self, target_altitude):
		local_position_of_takeoff_state = LocalPosition(self._start_state.local_position.north, self._start_state.local_position.east, target_altitude)
		takeoff_state = State(self._environment, self._goal_state.local_position, local_position_of_takeoff_state, parent_state=self._start_state)
		return takeoff_state

	def _determine_step_size_of_tree(self) -> float:
		step_size_of_tree = 2 * self._environment.obstacles.safety # experiment with this variable
		return step_size_of_tree

	def _determine_maximum_number_of_iterations(self) -> float:
		environment_north_length_in_meters = self._environment.north_bounds.maximum - self._environment.north_bounds.minimum
		environment_east_length_in_meters = self._environment.east_bounds.maximum - self._environment.east_bounds.minimum
		average_length = (environment_north_length_in_meters + environment_east_length_in_meters) / 2 
		maximum_number_of_iterations =  10 * average_length # Experiment with this number. Maybe make it a function of the distance between the start and goal states.
		return maximum_number_of_iterations

	def _take_a_sample(self, target_altitude, current_iteration) -> State:
		
		if current_iteration % 2 == 0 or current_iteration % 5 == 0: # Bias the sampler in favor of the goal state
			north = np.random.uniform(low=self._environment.north_bounds.minimum, high=self._environment.north_bounds.maximum)
			east = np.random.uniform(low=self._environment.east_bounds.minimum, high=self._environment.east_bounds.maximum)
		else:
			north = self._goal_state.local_position.north
			east = self._goal_state.local_position.east
			
		down = target_altitude
		local_position_of_random_sample = LocalPosition(north, east, down)
		random_sample_state = State(self._environment, self._goal_state.local_position, local_position_of_random_sample)

		return random_sample_state

	def _return_the_nearest_neighbor_state(self, random_sample_state: State) -> State:

		array_of_tree_positions = np.array([state.position_in_3d for state in self._sample_states])
		random_sample_position = random_sample_state.position_in_3d

		# Calculate the distances using NumPy's vectorized operations
		distances = np.linalg.norm(array_of_tree_positions - random_sample_position, axis=1)
		
		# Find the index of the minimum distance
		nearest_neighbor_index = np.argmin(distances)

		# Get the nearest neighbor state from the list
		nearest_neighbor_state = self._sample_states[nearest_neighbor_index]

		return nearest_neighbor_state

	def _determine_resolution_of_collision_detector(self) -> float:
		resolution_of_collision_detector = self._environment.obstacles.safety # meters
		return resolution_of_collision_detector

	def _sample_until_near_goal(self) -> List[State]:
		goal_is_found = False 
		current_state = self._takeoff_state

		while not goal_is_found:
			print("Attempting")
			current_iteration = 0
			while current_iteration < self._maximum_number_of_iterations and current_state.distance_to_goal > self._step_size_of_tree:
				random_sample_state = self._take_a_sample(self._takeoff_state.local_position.down, current_iteration)
				nearest_neighbor_state = self._return_the_nearest_neighbor_state(random_sample_state)
				vector_from_neighbor_to_random_sample = random_sample_state.position_in_3d - nearest_neighbor_state.position_in_3d
				length_of_the_vector_in_meters = np.linalg.norm(vector_from_neighbor_to_random_sample) 
				unit_vector = vector_from_neighbor_to_random_sample / length_of_the_vector_in_meters
				for distance_traversed in np.arange(self._resolution_of_collision_detector, self._step_size_of_tree, self._resolution_of_collision_detector):
					local_position_of_test_state_along_vector_step = nearest_neighbor_state.position_in_3d + distance_traversed * unit_vector
					local_position_of_test_state_along_vector_step = LocalPosition(*local_position_of_test_state_along_vector_step)
					state_to_test = State(self._environment, self._goal_state.local_position, local_position_of_test_state_along_vector_step, parent_state=nearest_neighbor_state)
					
					if self._environment.state_collides_with_obstacle(state_to_test):
						break

					if state_to_test.distance_to_goal <= self._step_size_of_tree:
						goal_is_found = True
						print('path is found')
						current_state = state_to_test
						break

					current_state = state_to_test

				self._sample_states.append(current_state)
				current_iteration += 1
		

	def _connect_goal_state(self):
		# This code block will add the goal state to the list of states if it can be connected to the final state before the goal state in a straight line without colliding
		# with obstacles. 
		
		final_state_before_goal = self._sample_states[-1]
		vector_from_end_of_tree_to_goal_state = self._goal_state.position_in_3d - final_state_before_goal.position_in_3d
		vector_mag = np.linalg.norm(vector_from_end_of_tree_to_goal_state)
		unit_vector = vector_from_end_of_tree_to_goal_state / vector_mag
		possible_to_connect_to_goal = True
		for displacement_towards_goal in np.arange(self._resolution_of_collision_detector, vector_mag, self._resolution_of_collision_detector):
			local_position_of_test_state_along_vector = final_state_before_goal.position_in_3d + displacement_towards_goal * unit_vector
			local_position_of_test_state_along_vector = LocalPosition(*local_position_of_test_state_along_vector)
			state_to_test = State(self._environment, self._goal_state.local_position, local_position_of_test_state_along_vector, parent_state=final_state_before_goal)
			test_state_is_in_collision = self._environment.state_collides_with_obstacle(state_to_test)
			if test_state_is_in_collision:
				print("Can't directly connect the goal state to the tree.")
				possible_to_connect_to_goal = False
				break
		if possible_to_connect_to_goal:
			self._goal_state.parent_state = final_state_before_goal
			self._sample_states.append(self._goal_state)
		else:
			print("It's not possible to connect the goal state to the random tree.")
		

	def _rewire_the_states(self) -> None:	
		# An algorithm to shorten the path
		for i in range(50): #Experiment with the number of times to shorten the path
			start = self._goal_state
			while start.parent_state is not None:
				if start.parent_state.parent_state is not None:
					subgoal = start.parent_state.parent_state 
					vector = subgoal.position_in_3d - start.position_in_3d
					vector_mag = np.linalg.norm(vector)
					unit_vector = vector / vector_mag
					for step in np.arange(self._resolution_of_collision_detector, vector_mag, self._resolution_of_collision_detector):
						local_position = start.position_in_3d + step * unit_vector
						local_position = LocalPosition(local_position[0], local_position[1], local_position[2])
						state = State(self._environment, self._goal_state.local_position, local_position)
						collides = self._environment.state_collides_with_obstacle(state)
						if collides:
							start = start.parent_state
							break
					if not collides:
						start.parent_state = subgoal
						start = subgoal
					if start == self._start_state:
						break
				else:
					break


	def _determine_path_of_states(self) -> StateCollection:
		# A loop to generate an list of states that is now the global plan.

		middle_states = []
		goal_state = self._goal_state
		current_state = self._goal_state
		while current_state.parent_state != self._start_state:
			middle_states.append(current_state.parent_state)
			current_state = current_state.parent_state
		middle_states = middle_states[::-1]
		path = StateCollection(self._start_state, self._goal_state, middle_states)
		
		cost = 0.0
		for state in path.list:
			if state.parent_state is not None:
				cost += np.linalg.norm(state.parent_state.position_in_3d - state.position_in_3d)

		print("The cost is", cost)
		return path, cost

	def visualize(self, plot_entire_state_space=False):
		# Plot the base environment
		fig, ax = self._environment.visualize()

		if plot_entire_state_space:
			for state in self._sample_states[2:]:
				plt.arrow(state.parent_state.local_position.north, 
					state.parent_state.local_position.east,
					state.local_position.north - state.parent_state.local_position.north,
					state.local_position.east - state.parent_state.local_position.east,
					head_width=4, head_length=4, length_includes_head=True, color="lime")

		for state in self._path.list[1:]:
			plt.text(state.local_position.north, state.local_position.east, f"{state.local_position.down: 0.1f} m", fontsize=8)
			plt.arrow(state.parent_state.local_position.north, 
				state.parent_state.local_position.east,
				state.local_position.north - state.parent_state.local_position.north,
				state.local_position.east - state.parent_state.local_position.east,
				head_width=4, head_length=4, length_includes_head=True, color='blue', linewidth=2)
		plt.text(self._path.list[0].local_position.north, self._path.list[0].local_position.east, f"{state.local_position.down: 0.1f} m", fontsize=8)
		
		explored_states_arrow = mpatches.FancyArrowPatch((0, 0), (0, 0), color="lime", lw=1)
		shortened_path_arrow = mpatches.FancyArrowPatch((0, 0), (0, 0), color="blue", lw=1)



		plt.scatter(self._start_state.local_position.north, self._start_state.local_position.east, color='green', marker='o', label='Start state')
		plt.scatter(self._goal_state.local_position.north, self._goal_state.local_position.east, color='red', marker='o', label='Goal state') 
		ax.set_xlim(self._environment.north_bounds.minimum, self._environment.north_bounds.maximum)
		ax.set_ylim(self._environment.east_bounds.minimum, self._environment.east_bounds.maximum)
		
		plt.legend(handles=[explored_states_arrow, shortened_path_arrow, mpatches.Patch(color='green', label='Start state'), mpatches.Patch(color='red', label='Goal state')], labels=['Explored states', 'Shortened path', 'Start state', 'Goal state'])

		plt.show()
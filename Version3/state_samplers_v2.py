from bounds import Bounds
from environment import Environment
from local_position import LocalPosition
import matplotlib.pyplot as plt
import numpy as np
from start_goal_pair import StartGoalPair
from state import State, PotentialMeshState
from state_collection import StateCollection
from typing import List


from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from matplotlib.patches import Circle
import matplotlib.patches as patches
import matplotlib.pyplot as plt

import pdb


class Planner:
	def __init__(self, environment: Environment, start_state: State, goal_state: State):
		#self._state_sequence = None # The solution of the planner's search routine, provided as a list or, if a solution can't be found, as the None type. 
		#self._search_space = None # A data structure generated by the planning algorithm to represent the entire state space, including its connectivity. 
		self._start_state = start_state
		self._goal_state = goal_state
		self._environment = environment
		self._state_sequence = None

	@property
	def state_sequence(self):
		return self._state_sequence

	@property
	def search_space(self):
		return self._search_space

	@property
	def start_state(self):
		return self._start_state

	@property
	def goal_state(self):
		return self._goal_state


class RapidlyExploringRandomTree(Planner):
	def __init__(self, environment: Environment, start_state: State, goal_state: State):
		super().__init__(environment, start_state, goal_state)

		self._environment = environment 
		self._start_state = start_state
		self._goal_state = goal_state
		self._middle_states = []
		self._tree_altitude = goal_state.local_position.down 
		self._takeoff_state = self._determine_takeoff_state(self._tree_altitude)
		self._state_sequence = self._build_tree_at_constant_altitude()

	
	def _determine_takeoff_state(self, target_altitude):
		local_position_of_takeoff_state = LocalPosition(self._start_state.local_position.north, self._start_state.local_position.east, target_altitude)
		takeoff_state = State(self._environment, self._goal_state.local_position, local_position_of_takeoff_state, parent_state=self._start_state)
		return takeoff_state

	def _determine_step_size_of_tree(self) -> float:
		step_size_of_tree = 2*self._environment.obstacles.safety # experiment with this variable
		return step_size_of_tree

	def _determine_maximum_number_of_iterations(self) -> float:
		environment_north_length_in_meters = self._environment.north_bounds.maximum - self._environment.north_bounds.minimum
		environment_east_length_in_meters = self._environment.east_bounds.maximum - self._environment.east_bounds.minimum
		average_length = (environment_north_length_in_meters + environment_east_length_in_meters) / 2 
		maximum_number_of_iterations =  1 * average_length # Experiment with this number. Maybe make it a function of the distance between the start and goal states.
		return maximum_number_of_iterations

	def _take_a_random_sample_state(self, target_altitude, current_iteration) -> State:
		if current_iteration % 2 == 0 or current_iteration % 5 == 0:
			north = np.random.uniform(low=self._environment.north_bounds.minimum, high=self._environment.north_bounds.maximum)
			east = np.random.uniform(low=self._environment.east_bounds.minimum, high=self._environment.east_bounds.maximum)
		else:
			north = self._goal_state.local_position.north
			east = self._goal_state.local_position.east
		down = target_altitude
		local_position_of_random_sample = LocalPosition(north, east, down)
		the_random_sample_state = State(self._environment, self._goal_state.local_position, local_position_of_random_sample)

		return the_random_sample_state

	def _return_the_nearest_neighbor_state(self, tree_states_as_list, the_random_sample_state) -> State:
		distance = np.inf
		the_nearest_neighbor_state = None
		for index, state in enumerate(tree_states_as_list):
			test_distance = np.linalg.norm(state.position_in_3d - the_random_sample_state.position_in_3d)
			if test_distance < distance:
				distance = test_distance
				the_nearest_neighbor_state = tree_states_as_list[index]

		return the_nearest_neighbor_state

	def _determine_resolution_of_collision_detector(self) -> float:
		resolution_of_collision_detector = self._environment.obstacles.safety # meters
		return resolution_of_collision_detector

	def _plot_solution(self):
		pass

	def _build_tree_at_constant_altitude(self) -> StateCollection:	

		# Establish the values of some key parameters.
		goal_is_found = False
		maximum_number_of_iterations = self._determine_maximum_number_of_iterations()
		step_size_of_tree = self._determine_step_size_of_tree()
		resolution_of_collision_detector = self._determine_resolution_of_collision_detector()
		tree_states_as_list = [self._start_state, self._takeoff_state]
		current_state = self._takeoff_state

		# Develop a collection of states that are connected in a tree structure, and store references to each state of the tree in a list. 
		while not goal_is_found:
			print("Attempting")
			current_iteration = 0
			while current_iteration < maximum_number_of_iterations and current_state.distance_to_goal > step_size_of_tree:
				the_random_sample_state = self._take_a_random_sample_state(self._takeoff_state.local_position.down, current_iteration)
				the_nearest_neighbor_state = self._return_the_nearest_neighbor_state(tree_states_as_list, the_random_sample_state)
				the_vector_from_neighbor_to_random_sample = the_random_sample_state.position_in_3d - the_nearest_neighbor_state.position_in_3d
				length_of_the_vector_in_meters = np.linalg.norm(the_vector_from_neighbor_to_random_sample) 
				the_unit_vector = the_vector_from_neighbor_to_random_sample / length_of_the_vector_in_meters
				for current_distance_traversed in np.arange(resolution_of_collision_detector, step_size_of_tree, resolution_of_collision_detector):
					local_position_of_test_state_along_vector_step = the_nearest_neighbor_state.position_in_3d + current_distance_traversed * the_unit_vector
					local_position_of_test_state_along_vector_step = LocalPosition(
						local_position_of_test_state_along_vector_step[0], 
						local_position_of_test_state_along_vector_step[1], 
						local_position_of_test_state_along_vector_step[2])
					state_to_test = State(self._environment, self._goal_state.local_position, local_position_of_test_state_along_vector_step, parent_state=the_nearest_neighbor_state)
					test_state_is_in_collision = self._environment.state_collides_with_obstacle(state_to_test)
					if test_state_is_in_collision:
						break
					if state_to_test.distance_to_goal <= step_size_of_tree:
						goal_is_found = True
						print('path is found')
						current_state = state_to_test
						break
					current_state = state_to_test
				tree_states_as_list.append(current_state)
				current_iteration += 1
		final_state_before_goal = current_state

		# Add the goal state to the state sequence if there's no collision between the goal state and the second to last
		# state. 
		vector = self._goal_state.position_in_3d - final_state_before_goal.position_in_3d
		vector_mag = np.linalg.norm(vector)
		unit_vector = vector / vector_mag
		possible_to_connect_to_goal = True
		for current_distance_traversed in np.arange(resolution_of_collision_detector, vector_mag, resolution_of_collision_detector):
			local_position_of_test_state_along_vector = final_state_before_goal.position_in_3d + current_distance_traversed * unit_vector
			local_position_of_test_state_along_vector = LocalPosition(
				local_position_of_test_state_along_vector[0],
				local_position_of_test_state_along_vector[1],
				local_position_of_test_state_along_vector[2])
			state_to_test = State(self._environment, self._goal_state.local_position, local_position_of_test_state_along_vector, parent_state=final_state_before_goal)
			test_state_is_in_collision = self._environment.state_collides_with_obstacle(state_to_test)
			if test_state_is_in_collision:
				print("Can't directly connect the goal state to the tree.")
				possible_to_connect_to_goal = False
				break
		if possible_to_connect_to_goal:
			self._goal_state.parent_state = final_state_before_goal
			tree_states_as_list.append(self._goal_state)
			

		#iteratively test parent states to cut down on cost
		#Try to rewire the tree if goal is found

		# Plot the base environment
		fig, ax = self._environment.visualize()

		# Plot the states that the RRT algorithm explored
		for state in tree_states_as_list[2:]:
			plt.arrow(state.parent_state.local_position.north, state.parent_state.local_position.east,
				state.local_position.north - state.parent_state.local_position.north,
				state.local_position.east - state.parent_state.local_position.east,
				head_width=4, head_length=4, color='black', length_includes_head=True)
		
		# An algorithm to shorten the path
		for i in range(25): #Experiment with the number of times to shorten the path
			start = final_state_before_goal
			while start.parent_state is not None:
				if start.parent_state.parent_state is not None:
					subgoal = start.parent_state.parent_state 
					vector = subgoal.position_in_3d - start.position_in_3d
					vector_mag = np.linalg.norm(vector)
					unit_vector = vector / vector_mag
					for step in np.arange(resolution_of_collision_detector, vector_mag, resolution_of_collision_detector):
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
					if start == self._takeoff_state:
						break
				else:
					break

		# A loop to generate an list of states that is now the global plan.
		state_sequence = []
		goal_state = self._goal_state
		current_state = self._goal_state
		while current_state.parent_state != None:
			state_sequence.append(current_state)
			current_state = current_state.parent_state
		state_sequence = state_sequence[::-1]
		state_sequence = StateCollection(self._start_state, self._goal_state, state_sequence)



		# Visualize the plan.		
		plt.text(self._takeoff_state.local_position.north, self._takeoff_state.local_position.east, f"{self._takeoff_state.local_position.down: 0.1f}", fontsize=12)
		# Plot the global plan state sequence
		for state in state_sequence.list[1:-1]:
			plt.text(state.local_position.north, state.local_position.east, f"{state.local_position.down: 0.1f}", fontsize=12)
			plt.arrow(
				state.parent_state.local_position.north, 
				state.parent_state.local_position.east,
				state.local_position.north - state.parent_state.local_position.north,
				state.local_position.east - state.parent_state.local_position.east,
				head_width=4, head_length=4, length_includes_head=True, color='green')
		
		# Plot the start and goal states
		plt.scatter(self._start_state.local_position.north, self._start_state.local_position.east, color='green', marker='o', label='Start state')
		plt.scatter(self._goal_state.local_position.north, self._goal_state.local_position.east, color='red', marker='o', label='Goal state') 
		ax.set_xlim(self._environment.north_bounds.minimum, self._environment.north_bounds.maximum)
		ax.set_ylim(self._environment.east_bounds.minimum, self._environment.east_bounds.maximum)
		plt.legend()

		plt.show()
		#pdb.set_trace()

		return state_sequence

	def visualize(self):
		pass

	def plot_environment(self):
		fig, ax = plt.subplots()
		ax.set_xlabel("North")
		ax.set_ylabel("East")
		color_map_of_plot = plt.get_cmap('coolwarm')
		list_of_obstacle_heights = [obstacle.height for obstacle in self._environment.obstacles.list]
		normalization_of_obstacle_heights = Normalize(vmin=min(list_of_obstacle_heights), vmax=max(list_of_obstacle_heights))
		scalar_mappable = ScalarMappable(norm=normalization_of_obstacle_heights, cmap=color_map_of_plot)
		scalar_mappable.set_array([])
		for obstacle in self._environment.obstacles.list:
			obstacle_color = color_map_of_plot(normalization_of_obstacle_heights(obstacle.height))
			obstacle_as_a_circle_patch = patches.Circle((obstacle.local_position.north, obstacle.local_position.east), obstacle.safety, color=obstacle_color, alpha=0.3)
			ax.add_patch(obstacle_as_a_circle_patch)
			ax.text(obstacle.local_position.north, obstacle.local_position.east, f"{float(obstacle.height):.1f}", fontsize=8)
		#ax.scatter(self.goal_state.local_position.north, self.goal_state.local_position.east, c='red', s=30, marker='x', label='Goal')
		#ax.legend()
		color_bar = plt.colorbar(scalar_mappable, ax=ax)
		color_bar.set_label("Altitude (m)")
		
	@property
	def state_sequence(self):
		return self._state_sequence

class LocalPositionVector:
	def __init__(self, north_component, east_component, down_component):
		self._north_component = north_component  
		self._east_component = east_component
		self._down_component = down_component 
	
	@property
	def north_component(self):
		return self._north_component

	@property
	def east_component(self):
		return self._east_component

	@property
	def down_component(self):
		return self._down_component

class ProbabilisticRoadmap(Planner):
	def visualize(self):
		pass

class PotentialField(Planner):

	def __init__(self, environment, start_state, goal_state):
		self._environment = environment
		self._start_state = start_state
		self._goal_state = goal_state
		#self._north_bounds = self._determine_north_bounds()
		#self._east_bounds = self._determine_east_bounds()
		#self._down_bounds = self._determine_down_bounds()
		self._resolution = 1 # meter cubed
		self._step_size = 0.1 # meter
		self._state_sequence = self._determine_path()


	def _compute_next_state(self, current_state: State) -> State:

		# Gains
		ks = 0
		ko = 10
		kg = 1000

		current_position = current_state.ground_position
		# Determine the position of the nearest obstacle
		nearest_obstacle_index = self._environment.obstacles.tree.query([current_position])[1][0][0]
		nearest_obstacle = self._environment.obstacles.list[nearest_obstacle_index]
		obstacle_position = np.array([nearest_obstacle.local_position.north,
									  nearest_obstacle.local_position.east])
		
		# Establish the start and goal positions of the potential field. Use some simplifying conditions for this prototype implementation. 
		goal_position = self._goal_state.ground_position
		start_position = self._start_state.ground_position

		# Calculate the attractive and repulsive forces "felt" at the current position
		vector_from_start_to_current = current_position - start_position
		distance_start_to_current = np.linalg.norm(vector_from_start_to_current)
		unit_vector_start_to_current = vector_from_start_to_current / distance_start_to_current
		force_start_to_current = 0 #Ignore for now: ks / distance_start_to_current**2 * unit_vector_start_to_current

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
		new_position = current_position + self._step_size * unit_vector_at_current
		new_position = LocalPosition(new_position[0], new_position[1], 10) #TAKEOFF ALTITUDE
		
		new_state = State(self._environment, self._goal_state.local_position, new_position, heading=0, parent_state=None)
		return new_state

	def _determine_path(self) -> List[State]:
		# Search the graph of potential field states for the minimum cost path. Or the path that is monotonically decreasing. 
		
		trajectory = []
		current_state = self._start_state
		
		for _ in range(10000):
			print(current_state.position_in_3d)
			new_state = self._compute_next_state(current_state)
			trajectory.append(new_state)
			if np.linalg.norm(self._goal_state.position_in_3d - new_state.position_in_3d) <= self._step_size:
				return trajectory
			current_state = new_state
			#pdb.set_trace()
		

		return trajectory

	def visualize(self):
		# Put all the potential states on the map, labeled with their potential values, having a colormap of their own to indicate their total potential.
		pass

	@property
	def state_sequence(self):
		return self._state_sequence

class Grid(Planner):
	def visualize(self):
		pass

class Voronoi(Planner):
	def visualize(self):
		pass

class MedialAxis(Planner):
	def visualize(self):
		pass

class Voxel(Planner):
	def visualize(self):
		pass

"""
Each planner will return a sequence of states from start to goal based on some specific algorithm. 
All planners will return a sequence of states from start to goal. If no goal can be found, then 
the algorithm will retry put the drone in a hover state. 
 
"""
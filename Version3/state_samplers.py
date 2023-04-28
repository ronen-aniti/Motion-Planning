from bounds import Bounds
from environment import Environment
from local_position import LocalPosition
import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from start_goal_pair import StartGoalPair
from state import State 
from state_collection import StateCollection
from queue import PriorityQueue
from typing import List


import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
import matplotlib.patches as patches


import pdb

class StateSampler:
	def __init__(self, pairing: StartGoalPair):
		self._pairing = pairing

	def convert_state_to_waypoint(state) -> np.ndarray:
		pass

	@property
	def start_goal_pair(self):
		return self._pairing

class LocalSampler(StateSampler):
	def __init__(self, pairing: StartGoalPair):
		super().__init__(pairing)

	def determine_next_state(self, environment: Environment, num_samples: int):
		#Conduct A* search through samples. The minimum cost path should be directly from start to goal. However,
		#if a new obstacle blocks that path, the A* search would plan around the block.
		#return next state to fly to using local algo
		pass

class GlobalSampler(StateSampler):
	def __init__(self, pairing: StartGoalPair):
		super().__init__(pairing)

	def determine_global_state_sequence_new(self, environment: Environment, desired_num_samples: int) -> List[State]:
		# Implement a rapidly-exploring random tree

		samples_so_far = 0
		states = []
		while samples_so_far < desired_num_samples:
			north = np.random.uniform(low=environment.north_bounds.minimum, high=environment.north_bounds.maximum)
			east = np.random.uniform(low=environment.east_bounds.minimum, high=environment.east_bounds.maximum)
			down_mean, down_stdev = 10, 2
			down = np.random.normal(down_mean, down_stdev, 1) # Sample states around a target altitude
			ground_position_of_sample = np.array([north, east])


	def determine_state_sequence(self, start_state: State, goal_state: State, environment: Environment) -> List[State]:

		# Sample states for global path planning
		samples_so_far = 0
		middle_states = []
		desired_num_samples = 500 # Think about how the desired number of samples should relate to the size of the sample space

		# Compute the dimensions of a bounding box for the sample routine
		ground_distance_between_start_and_goal_states = np.linalg.norm(start_state.ground_position - goal_state.ground_position)
		vertical_distance_between_start_and_goal_states = abs(start_state.local_position.down - goal_state.local_position.down)
		k=1
		bounding_box_north_minimum = start_state.local_position.north - k * ground_distance_between_start_and_goal_states
		bounding_box_north_maximum = start_state.local_position.north + k * ground_distance_between_start_and_goal_states
		bounding_box_east_minimum = start_state.local_position.east - k * ground_distance_between_start_and_goal_states
		bounding_box_east_maximum = start_state.local_position.east  + k * ground_distance_between_start_and_goal_states
		bounding_box_down_minimum = start_state.local_position.down - k * vertical_distance_between_start_and_goal_states
		bounding_box_down_maximum = start_state.local_position.down + k * vertical_distance_between_start_and_goal_states
		if bounding_box_north_minimum < environment.north_bounds.minimum:
			bounding_box_north_minimum = environment.north_bounds.minimum
		if bounding_box_north_maximum > environment.north_bounds.maximum:
			bounding_box_north_maximum = environment.north_bounds.maximum
		if bounding_box_east_minimum < environment.east_bounds.minimum:
			bounding_box_east_minimum = environment.east_bounds.minimum
		if bounding_box_east_maximum > environment.east_bounds.maximum:
			bounding_box_east_maximum = environment.east_bounds.maximum
		if bounding_box_down_minimum < environment.down_bounds.minimum:
			bounding_box_down_minimum = environment.down_bounds.minimum
		if bounding_box_down_maximum > environment.down_bounds.maximum:
			bounding_box_down_maximum = environment.down_bounds.maximum
		north_sample_bounds = Bounds(bounding_box_north_minimum, bounding_box_north_maximum)
		east_sample_bounds = Bounds(bounding_box_east_minimum, bounding_box_east_maximum)
		down_sample_bounds = Bounds(bounding_box_down_minimum, bounding_box_down_maximum)

		# Generate sample states within the bounds of the environment that aren't occupied by obstacles
		while samples_so_far < desired_num_samples:
			# Generate a sample state coordinate
			north = np.random.uniform(low=north_sample_bounds.minimum, high=north_sample_bounds.maximum)
			east = np.random.uniform(low=east_sample_bounds.minimum, high=east_sample_bounds.maximum)
			down = goal_state.local_position.down

			ground_position_of_sample = np.array([north, east])

			# Reject samples that aren't unoccupied positions
			sample_is_within_ground_distance_of_obstacle = False
			sample_is_in_collision_with_obstacle = False
			neighbor_obstacle_indices_array = environment.obstacles.tree.query_radius([ground_position_of_sample], r=environment.obstacles.safety)[0]
			if len(neighbor_obstacle_indices_array) > 0:
				sample_is_within_ground_distance_of_obstacle = True
			if sample_is_within_ground_distance_of_obstacle:
				for neighbor_obstacle_index in neighbor_obstacle_indices_array:
					if environment.obstacles.list[neighbor_obstacle_index].height > down:
						sample_is_in_collision_with_obstacle = True
				if not sample_is_in_collision_with_obstacle:
					local_position = LocalPosition(north, east, down)
					middle_states.append(State(environment, local_position))
					samples_so_far += 1

		# Collect the states into a single object
		state_collection = StateCollection(start_state, middle_states, goal_state) 

		# Make a graph from the collection of states. Connect nodes whose connection vector doesn't collide with any obstacles.
		state_graph = nx.Graph()
		state_graph.add_nodes_from(state_collection.list)

		# For every state in the state collection list, create a two-element tuple with the state's five nearest neighbors, using ground distance as the proximity metric.
		weighted_edges = []
		num_neighbors_to_consider_for_edges = 25 #int(desired_num_samples/30) # the number of neighbor states to consider scales in proportion to the number of states
		## We must add one to the num_neighbors_to_consider_for_edges so that we don't count the origin state itself.
		num_neighbors_to_query = num_neighbors_to_consider_for_edges + 1
		for state in state_collection.list:
			neighbor_state_distances, neighbor_state_indices = state_collection.tree.query([state.position_in_3d], k=num_neighbors_to_query)
			# Iterate through the current state's neighboring states
			# The zero index reference is needed since the arrays returned from the query function are nested.
			for neighbor_state_distance, neighbor_state_index in zip(neighbor_state_distances[0], neighbor_state_indices[0]):
				neighbor_state = state_collection.list[neighbor_state_index] # Establish the neighboring state to traverse to in the lines that follow.
				if np.all(state.position_in_3d == neighbor_state.position_in_3d):
					continue
				displacement_vector = neighbor_state.position_in_3d - state.position_in_3d
				displacement_vector_magnitude = np.linalg.norm(displacement_vector)
				unit_direction_vector = displacement_vector/displacement_vector_magnitude
				edge_is_valid = True
				distance_step_size = 5
				distances_along_direction = np.arange(0, displacement_vector_magnitude, 5)
				# One a neighboring state is selected, traverse from the original state to the neighbor state using a vector equation,
				# creating test states along the way, testing each test state for collision with obstacles. As soon as a collision is 
				# detected, forget that neighbor state as an edge connection option, go on, and start traversing to the next neighbor state. 
				for distance in distances_along_direction:
					# Assume test state isn't in collision with an obstacle before it's tested whether or not it is so.
					test_state_is_in_collision = False 
					# Establish a 3d position for the test state
					test_state_3d_position = state.position_in_3d + distance * unit_direction_vector 
					# Build a LocalPosition object from that 3d position
					local_position_of_test_state = LocalPosition(*test_state_3d_position) 
					# Build a State object from that local position
					test_state = State(environment, local_position_of_test_state) 
					# Ask: Does this test state collide with any obstacles?
					obstacle_neighbor_indices_array = environment.obstacles.tree.query_radius([test_state.ground_position], r=environment.obstacles.safety)[0]
					test_state_is_within_ground_distance_of_obstacle = len(obstacle_neighbor_indices_array) > 0
					# If it doesn't then continue testing more states along the displacement vector between the start state and the neighbor state.
					# But if it does, break the vector travel loop so the next neighbor state can be tested
					# Also, every test state along the vector travel loop doesn't collide with an obstacle, then add the start state and the neighbor state
					# as elements of a sublist, and append it to the edge list that will define the edges of a connected graph. 
					if test_state_is_within_ground_distance_of_obstacle:
						for obstacle_neighbor_index in obstacle_neighbor_indices_array:
							if environment.obstacles.list[obstacle_neighbor_index].height > test_state.local_position.down:
								test_state_is_in_collision = True
								break
					if test_state_is_in_collision:
						edge_is_valid = False
						break

				if edge_is_valid:	
					distance_between_states = np.linalg.norm(displacement_vector)
					weighted_edges.append((state, neighbor_state, distance_between_states))
		
		# Add the weighted edges to the graph of states to establish the connectivity of the state space.
		state_graph.add_weighted_edges_from(weighted_edges)
		
		print('done calculating...')


		
		state_sequence = []
		current_state = start_state
 		queue_of_states_to_access_next = PriorityQueue()
		queue_of_states_to_access_next.put((start_state, 0))
		visited_states = {start_state}
		

		iterator_for_neighbor_states_to_current_state = state_graph.neighbors(current_state)
		for neighbor_state in iterator_for_neighbor_states_to_current_state:
			if neighbor_state not in visited_states:


		"""
		# Search the graph of states for the minimum-cost path using A*, and euclidean distance as the heuristic. 
		print('searching the graph of the sampled states to determine the minimum-cost path between the start state and the goal state')

		visited_states = set()
		state_sequence = []
		queue_of_states_to_access_next = PriorityQueue()
		queue_of_states_to_access_next.put((start_state, 0))
		current_state = start_state


		while not queue_of_states_to_access_next.empty():

			current_state = queue_of_states_to_access_next.get()[0] # Gets the state that has the lowest total cost
			print(current_state)
			visited_states.add(current_state)
			state_sequence.append(current_state)

			if current_state == goal_state:
				print("A path from the start state to the goal state has been found.")
				break

			iterator_for_neighbor_states_to_current_state = state_graph.neighbors(current_state)
			minimum_possible_cost_of_next_move = np.inf
			for neighbor_state in iterator_for_neighbor_states_to_current_state:
				if neighbor_state not in visited_states:
					vector_from_neighbor_state_to_goal_state = goal_state.position_in_3d - neighbor_state.position_in_3d
					distance_from_neighbor_state_to_goal_state = np.linalg.norm(vector_from_neighbor_state_to_goal_state)
					distance_from_current_state_to_neighbor_state = state_graph.get_edge_data(current_state, neighbor_state)['weight']
					total_cost_of_accessing_this_neighbor = distance_from_current_state_to_neighbor_state + distance_from_neighbor_state_to_goal_state
					if total_cost_of_accessing_this_neighbor < minimum_possible_cost_of_next_move:
						minimum_possible_cost_of_next_move = total_cost_of_accessing_this_neighbor
						neighbor_state_having_minimum_cost_move = neighbor_state
			if minimum_possible_cost_of_next_move == np.inf:
				print("There's not a path between the start state and the goal state.")
				break
			queue_of_states_to_access_next.put((neighbor_state_having_minimum_cost_move, minimum_possible_cost_of_next_move))

		print(visited_states)
		print(len(visited_states))
		print(len(state_sequence))
		self.plot_2d(environment, state_graph, start_state, goal_state, state_sequence)
		"""








		# For every incremental distance in the direction from the origin state to the neighbor state, create a new test state.
		# If that test state is in collision with an obstacle, then don't add an edge sublist [state1, state2] to the edge list
		# If I'm able to traverse the entire straight-line distance without colliding with an obstacle, then I do add [state1, state2] to the edge list
		# After this step, I add all the edges from the edge list to the state graph

		# After that, I will be able to write a graph search method to traverse from start state to goal state, and that will
		# serve as my global planner, or my global state sequence method. I'll return that list of states to main. 


		pdb.set_trace()

	def convert_state_sequence_to_waypoints(global_state_sequence: List[State]) -> List[float]:
		
		"""
		Ex. 
		local_sampler = StateSampler(local_start_goal_pair, 200, 'local')
		next_state = local_sampler.search(whole_environment)
		"""
		pass

	def plot_2d(self, environment: Environment, state_graph: nx.Graph, start_state, goal_state, state_sequence):

		fig, ax = plt.subplots()

		# Set the axis labels
		ax.set_xlabel('North')
		ax.set_ylabel('East')

		# Set the colormap and normalization
		cmap = plt.get_cmap('coolwarm')
		elevations = [obstacle.height for obstacle in environment.obstacles.list]
		norm = Normalize(vmin=min(elevations), vmax=max(elevations))
		sm = ScalarMappable(norm=norm, cmap=cmap)
		sm.set_array([])  # Set an empty array for the ScalarMappable object

		# Plot the obstacles
		for obstacle in environment.obstacles.list:
			obstacle_color = cmap(norm(obstacle.height))  # Get the color for the obstacle based on its height
			circle = patches.Circle((obstacle.local_position.north, obstacle.local_position.east), obstacle.safety, color=obstacle_color, alpha=0.3)
			ax.add_patch(circle)
			ax.text(obstacle.local_position.north, obstacle.local_position.east, f"{float(obstacle.height):.1f}", fontsize=8)

		# Plot the states and edges
		for state in state_graph.nodes:
			neighbors = state_graph.neighbors(state)

			for neighbor in neighbors:
				# Calculate the average elevation of the connected states
				avg_elevation = (state.local_position.down + neighbor.local_position.down) / 2

				# Get the color based on the average elevation
				edge_color = cmap(norm(avg_elevation))
				edge_color = (edge_color[0], edge_color[1], edge_color[2], 1.0)  # Set alpha to 1.0

				# Plot the edge with the determined color
				ax.plot([state.local_position.north, neighbor.local_position.north],
						[state.local_position.east, neighbor.local_position.east], c='black', alpha=0.7)


			# Plot the state node
			state_color = cmap(norm(state.local_position.down))#[:3]  # extract the RGB tuple from the RGBA tuple
			ax.scatter(state.local_position.north, state.local_position.east, c=state_color, s=10)
			ax.text(state.local_position.north, state.local_position.east, f"{float(state.local_position.down):.1f}", fontsize=8)




		start_color = 'red'
		goal_color = 'red'
		ax.scatter(start_state.local_position.north, start_state.local_position.east, c=start_color, s=30, marker='o', label='Start')
		ax.scatter(goal_state.local_position.north, goal_state.local_position.east, c=goal_color, s=30, marker='x', label='Goal')
		ax.legend()

		# Add a colorbar
		cb = plt.colorbar(sm, ax=ax)
		cb.set_label("Elevation (down)")

		plt.show()


from environment import Environment
from local_position import LocalPosition
import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from start_goal_pair import StartGoalPair
from state import State 
from state_collection import StateCollection
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

	def determine_global_state_sequence(self, environment: Environment, desired_num_samples: int) -> List[State]:
		#Implement A* search from start state to goal state given the environment and a sample size.
		#Use a uniform random distribution to generate samples that don't have any obstacles within the 
		#environment's obstacle collection's safety radius. Return the list of states that the drone should
		#plan to fly through. 
		
		# Sample states for global path planning
		samples_so_far = 0
		states = []
		while samples_so_far < desired_num_samples:
			north = np.random.uniform(low=environment.north_bounds.minimum, high=environment.north_bounds.maximum)
			east = np.random.uniform(low=environment.east_bounds.minimum, high=environment.east_bounds.maximum)
			down_mean, down_stdev = 10, 2
			down = np.random.normal(down_mean, down_stdev, 1) # Sample states around a target altitude
			ground_position_of_sample = np.array([north, east])
			neighbor_obstacle_indices_array = environment.obstacles.tree.query_radius([ground_position_of_sample], r=environment.obstacles.safety)[0]
			sample_is_within_ground_distance_of_obstacle = len(neighbor_obstacle_indices_array) > 0 
			sample_is_valid = True
			if sample_is_within_ground_distance_of_obstacle:
				for neighbor_obstacle_index in neighbor_obstacle_indices_array:
					if environment.obstacles.list[neighbor_obstacle_index].height > down:
						sample_is_valid = False
			if sample_is_valid:
				local_position = LocalPosition(north, east, down)
				states.append(State(environment, local_position))
				samples_so_far += 1

		state_collection = StateCollection(states)

		# Add the states as nodes in a graph
		state_graph = nx.Graph()
		state_graph.add_nodes_from(state_collection.list)

		# For every state in the state collection list, create a two-element tuple with the state's five nearest neighbors, using ground distance as the proximity metric.
		weighted_edges = []
		num_neighbors_to_consider_for_edges = 25
		## We must add one to the num_neighbors_to_consider_for_edges so that we don't count the origin state itself.
		num_neighbors_to_query = num_neighbors_to_consider_for_edges + 1
		for state in state_collection.list:
			sublist = ()
			neighbor_state_distances, neighbor_state_indices = state_collection.tree.query([state.position_in_3d], k=num_neighbors_to_query)
			# Iterate through the current state's neighboring states
			# The zero index reference is needed since the arrays returned from the query function are nested.
			for neighbor_state_distance, neighbor_state_index in zip(neighbor_state_distances[0], neighbor_state_indices[0]):
				neighbor_state = state_collection.list[neighbor_state_index] # Establish the neighboring state to traverse to in the lines that follow.
				if state == neighbor_state:
					continue
				displacement_vector = neighbor_state.position_in_3d - state.position_in_3d
				edge_is_valid = True
				parameter_0_to_1 = np.arange(0, 1, 0.01)
				# One a neighboring state is selected, traverse from the original state to the neighbor state using a vector equation,
				# creating test states along the way, testing each test state for collision with obstacles. As soon as a collision is 
				# detected, forget that neighbor state as an edge connection option, go on, and start traversing to the next neighbor state. 
				for parameter_step in parameter_0_to_1:
					# Assume test state isn't in collision with an obstacle before it's tested whether or not it is so.
					test_state_is_in_collision = False 
					# Establish a 3d position for the test state
					test_state_3d_position = state.position_in_3d + parameter_step * displacement_vector 
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
		self.plot_2d(environment, state_graph)




		pdb.set_trace()









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

	def plot_2d(self, environment, state_graph):
		fig, ax = plt.subplots()

		# Set the axis labels
		ax.set_xlabel('North')
		ax.set_ylabel('East')
		
		# Set the colormap and normalization
		cmap = plt.get_cmap('viridis')
		elevations = [state.local_position.down for state in state_graph.nodes]
		norm = Normalize(vmin=min(elevations), vmax=max(elevations))
		sm = ScalarMappable(norm=norm, cmap=cmap)
		sm.set_array([])  # Set an empty array for the ScalarMappable object


		# Plot the obstacles
		for obstacle in environment.obstacles.list:
			obstacle_color = cmap(norm(obstacle.height))  # Get the color for the obstacle based on its height
			circle = patches.Circle((obstacle.local_position.north, obstacle.local_position.east), obstacle.safety, color=obstacle_color, alpha=0.3)
			ax.add_patch(circle)

		# Plot the states and edges

		for state in state_graph.nodes:
			neighbors = state_graph.neighbors(state)
			color = tuple(cmap(norm(state.local_position.down))[0])  # Convert the color array to a tuple
			for neighbor in neighbors:
				ax.plot([state.local_position.north, neighbor.local_position.north],
						[state.local_position.east, neighbor.local_position.east],
						color=color, alpha=0.7)  # Use the color tuple

			ax.scatter(state.local_position.north, state.local_position.east, c=color, s=10)  # Use the color tuple
			ax.text(state.local_position.north, state.local_position.east, f"{state.local_position.down[0]:.1f}", fontsize=8)  # Access the first element of the numpy array

		# Add a colorbar
		cb = plt.colorbar(sm, ax=ax)
		cb.set_label("Elevation (down)")

		plt.show()

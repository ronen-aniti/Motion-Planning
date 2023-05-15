from environment import Environment
import matplotlib.pyplot as plt
from state import State
from local_position import LocalPosition
from planner import Planner
from queue import PriorityQueue
from typing import List, Tuple, Dict
from sklearn.neighbors import KDTree
from state import State
from state_collection import StateCollection
import pdb
import numpy as np
import copy

class ProbabilisticRoadmap(Planner):
	# Generate a random state that's not occupied by an obstacle 
	# Repeat N number of times
	# Sample the goal state
	# For each state that I sampled do the following.
	# 1. Consider its n nearest neighbor states
	# 2. For each neighbor state ask: could a straight line of states of resolution X be drawn to connect the start and end states?
	# 3. If so, then add an edge between state 1 and state 2. Keep track of edges using a dictionary.
	# Conduct an A* solution path from the start state to the goal state
	# Return the list of waypoints and the cost. 
	def __init__(self, environment: Environment, start_state: State, goal_state: State):
		super().__init__(environment, start_state, goal_state)
		self._number_of_samples = self._determine_number_of_samples()
		self._sample_heights = goal_state.local_position.down
		self._sample_list = self._take_samples()
		self._sample_kdtree = self._kdtree_from_state_list()
		self._valid_edge_list = self._determine_all_valid_edges() # List of tuples of pointer to state 1 and pointer to state 2
		self._graph = self._build_graph_from_edge_list()

	def _determine_number_of_samples(self) -> int:
		number_of_samples = int(max(self._environment.north_bounds.maximum, self._environment.east_bounds.maximum))/6
		return number_of_samples

	def _take_samples(self) -> List[State]:
		number_of_valid_samples = 0
		sample_list = [self._start_state, self._goal_state]
		while number_of_valid_samples < self._number_of_samples:
			north = np.random.uniform(low=self._environment.north_bounds.minimum, high=self._environment.north_bounds.maximum)
			east = np.random.uniform(low=self._environment.east_bounds.minimum, high=self._environment.east_bounds.maximum)
			down = self._sample_heights
			sample_state = State(self._environment, self._goal_state.local_position, LocalPosition(north, east, down))
			if self._environment.state_collides_with_obstacle(sample_state):
				continue
			
			print(sample_state.position_in_3d)
			sample_list.append(sample_state)
			number_of_valid_samples += 1
		return sample_list

	def _kdtree_from_state_list(self) -> KDTree:
		# Build a KDTree that maps a sample state position, in R3, to an index of the self._sample_list collection.
		state_positions = [state.ground_position for state in self._sample_list]
		state_kdtree = KDTree(state_positions)
		return state_kdtree

	def _determine_all_valid_edges(self) -> List[Tuple[State, State]]:
		valid_edge_list = []
		for state in self._sample_list:
			print("Connecting edges")
			# Retrieve its 7 nearest neighbors
			distances, indices = self._sample_kdtree.query([state.ground_position], k=20)			
			# For each, determine if a connection can be made. Append a tuple of state pointers to represent the connection if so. 
			neighbor_states = [self._sample_list[i] for i in indices[0]]

			for neighbor_state in neighbor_states:
				if self._is_straight_line_path_free(state, neighbor_state):
					valid_edge_list.append((state, neighbor_state))

		return valid_edge_list

	def _is_straight_line_path_free(self, state1: State, state2: State) -> bool:
		"""Determines if there's an obstacle blocking the straight line between two states"""
		resolution = self._environment.obstacles.safety # meters
		vector_between_states = state2.position_in_3d - state1.position_in_3d
		vector_mag = np.linalg.norm(vector_between_states)
		if vector_mag == 0:
			return False
		unit_vector = vector_between_states / vector_mag
		for step in np.arange(resolution, vector_mag, resolution):
			test_position = state1.position_in_3d + unit_vector * step
			test_state = State(self._environment, self._goal_state.local_position, LocalPosition(*test_position))
			if self._environment.state_collides_with_obstacle(test_state):
				return False
		return True


	def _build_graph_from_edge_list(self) -> Dict[State, State]:
		# Key: State 1 pointer. Value: State 2 pointer. Create a key for state 1. Set its value to state 2. Create a key for state 2. Set its value to state 1.
		graph = {}
		for state1, state2 in self._valid_edge_list:
			if state1 not in graph:
				graph[state1] = []
			if state2 not in graph:
				graph[state2] = []
			graph[state1].append(state2)
			graph[state2].append(state1)

		return graph

	def determine_waypoints(self) -> List[List[float]]:
		# Use A* to search graph from start to goal
		start_state = self._start_state
		goal_state = self._goal_state

		states_to_explore = PriorityQueue()

		states_to_explore.put((0.0, start_state))
		visited = set()
		visited.add(start_state)
		path = {}
		goal_is_found = False

		while not states_to_explore.empty():  
			print("IN LOOP")
			current_state = states_to_explore.get()[1]			

			if current_state == goal_state:
				goal_is_found = True
				break

			for state_to_visit in self._graph[current_state]:
				if state_to_visit not in visited:
					step_cost = np.linalg.norm(state_to_visit.position_in_3d - current_state.position_in_3d)
					heuristic_cost = np.linalg.norm(state_to_visit.position_in_3d - goal_state.position_in_3d)
					priority_cost = step_cost + heuristic_cost
					states_to_explore.put((priority_cost, state_to_visit))
					visited.add(state_to_visit)
					path[state_to_visit] = current_state


		if goal_is_found: 
			print("Goal is found.")
			# Reconstruct path
			current_state = goal_state
			reconstructed_path = [current_state]
			while current_state is not start_state:
				current_state.parent_state = path[current_state]
				reconstructed_path.append(path[current_state])
				current_state = path[current_state]
			reconstructed_path = reconstructed_path[::-1]
			self._path = reconstructed_path
			waypoints = [[state.local_position.north, state.local_position.east, state.local_position.down, state.heading] for state in self._path]

			return waypoints

		self._take_samples()




	def _determine_cost(self) -> float:
		# Return the total straight line distance between subsequent waypoints
		pass


	def visualize(self, plot_entire_state_space=True):
		# Plot the base environment
		fig, ax = self._environment.visualize()
		if  plot_entire_state_space:
			for state1, state2_list in self._graph.items():
				for state2 in state2_list:
					plt.plot([state1.local_position.north, state2.local_position.north],
							 [state1.local_position.east, state2.local_position.east],
							 color="lime", linewidth=0.3, marker='o', linestyle='-', label="Probabilistic roadmap")	

		plt.plot([state.local_position.north for state in self._path], [state.local_position.east for state in self._path], color="blue", label="Path")
		plt.scatter(self._start_state.local_position.north, self._start_state.local_position.east, color='green', marker='o', label='Start state')
		plt.scatter(self._goal_state.local_position.north, self._goal_state.local_position.east, color='red', marker='o', label='Goal state') 
		ax.set_xlim(self._environment.north_bounds.minimum, self._environment.north_bounds.maximum)
		ax.set_ylim(self._environment.east_bounds.minimum, self._environment.east_bounds.maximum)
		ax.legend()
		plt.show()
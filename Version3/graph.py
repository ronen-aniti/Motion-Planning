from enum import Enum
from environment import Environment
from state import State
from local_position import LocalPosition
import matplotlib.pyplot as plt
from planner import Planner
from typing import List
import pdb
import numpy as np
import copy
from queue import PriorityQueue
from scipy.spatial import KDTree



class Graph(Planner):
	def __init__(self, environment, start_state, goal_state):
		super().__init__(environment, start_state, goal_state)
		self._graph_altitude = self._goal_state.local_position.down
		self._resolution = 20
		self._states_kdtree = None
		self._states = None
		self._edges = self._build_uniform_mesh()
		
	def _build_uniform_mesh(self):
		edges = {}
		resolution = self._resolution
		north_positions = np.arange(self._environment.north_bounds.minimum, self._environment.north_bounds.maximum + resolution, resolution)
		east_positions = np.arange(self._environment.east_bounds.minimum, self._environment.east_bounds.maximum + resolution, resolution)
		down = self._goal_state.local_position.down
		self._states = []
		for north_position in north_positions:
			for east_position in east_positions:			
				test_state = State(self._environment, self._goal_state.local_position, LocalPosition(north_position, east_position, down))
				if not self._environment.state_collides_with_obstacle(test_state):
					print(test_state.ground_position)
					self._states.append(test_state)
		self._states_kdtree = KDTree([state.ground_position for state in self._states]) #index the states list
		for state in self._states:
			distances, indices = self._states_kdtree.query(state.ground_position, k=9)
			nine_states = [self._states[i] for i in indices]
			state_of_interest = nine_states[0]
			neighbor_states = [state for state in nine_states[1:] if np.linalg.norm(state_of_interest.ground_position - state.ground_position) <= self._resolution * np.sqrt(2)]
			edges[state_of_interest] = neighbor_states
		print(edges)
		return edges

	def visualize(self, plot_entire_state_space=True):
		self._environment.visualize()
		if plot_entire_state_space:
			for state, neighbor_states in self._edges.items():
				plt.scatter(state.local_position.north, state.local_position.east, color="lime", s=10)
				N = [neighbor_state.local_position.north for neighbor_state in neighbor_states]
				E = [neighbor_state.local_position.east for neighbor_state in neighbor_states]
				for i in range(len(neighbor_states)):
					plt.plot([state.local_position.north, N[i]], [state.local_position.east, E[i]], color="orange", linewidth=0.5)

		plt.plot([state.local_position.north for state in self._sequence],[state.local_position.east for state in self._sequence], "blue", alpha=1)
		plt.show()

	def search(self) -> np.ndarray:
		current_position = self._start_state.ground_position
		start_state_index = self._states_kdtree.query([current_position], k=1)[1][0]
		start_state = self._states[start_state_index]
		goal_position = self._goal_state.ground_position
		goal_state_index = self._states_kdtree.query([goal_position], k=1)[1][0]
		goal_state = self._states[goal_state_index]

		branch = {}
		visited = set()
		goal_is_found = False
		queue = PriorityQueue()
		queue.put((0.0, start_state))

		while not queue.empty():

			current_state = queue.get()[1]
			visited.add(current_state)
			
			if current_state is goal_state:
				goal_is_found = True
				print("The goal has been found.")
				break


			for neighbor_state in self._edges[current_state]:
				if neighbor_state not in visited:
					g_score = np.linalg.norm(current_state.ground_position - neighbor_state.ground_position)
					h_score = np.linalg.norm(neighbor_state.ground_position - goal_state.ground_position)
					f_score = g_score + h_score
					queue.put((f_score, neighbor_state))		
					branch[neighbor_state] = (current_state, g_score)
					visited.add(neighbor_state)

		if goal_is_found: 
			cost = 0.0
			path = [goal_state]
			current_state = goal_state
			while current_state is not start_state:
				cost += branch[current_state][1]
				path.append(branch[current_state][0])
				current_state = branch[current_state][0]
			path.reverse()
			print(path)
			print(("The cost is", cost))
			self._sequence = path
			return [state.waypoint for state in path]
		else:
			print("A* failed to find a path.")
			return None





from enum import Enum
from environment import Environment
from state import State
from local_position import LocalPosition
import matplotlib.pyplot as plt
from planner import Planner
from typing import List, Tuple
import pdb
import numpy as np
import copy
from queue import PriorityQueue
import scipy.ndimage
from scipy.spatial import KDTree


class Grid(Planner):
	def __init__(self, environment: Environment, start_state: State, goal_state: State):
		super().__init__(environment, start_state, goal_state)
		self._grid, self._north_offset, self._east_offset = self._convert_obstacles_to_grid()
		self._start_grid_north = int(self._start_state.local_position.north - self._north_offset)
		self._start_grid_east = int(self._start_state.local_position.east - self._east_offset)
		self._goal_grid_north = int(self._goal_state.local_position.north - self._north_offset)
		self._goal_grid_east = int(self._goal_state.local_position.east - self._east_offset)
		self._start_grid = (self._start_grid_north, self._start_grid_east)
		self._goal_grid = (self._goal_grid_north, self._goal_grid_east)

	def _convert_obstacles_to_grid(self):
		altitude = self._goal_state.local_position.down
		self._north_size = int(self._environment.north_bounds.maximum - self._environment.north_bounds.minimum)
		self._east_size = int(self._environment.east_bounds.maximum - self._environment.east_bounds.minimum)
		north_offset = self._environment.obstacles.north_offset
		east_offset = self._environment.obstacles.east_offset
		grid = np.zeros((self._north_size, self._east_size))
		for obstacle in self._environment.obstacles.list:
			north_start = int(obstacle.north_min - north_offset)
			north_end = int(obstacle.north_max - north_offset)
			east_start = int(obstacle.east_min - east_offset)
			east_end = int(obstacle.east_max - east_offset)

			grid[north_start: north_end, east_start:east_end] = 1.0 # occupied
		
		distance_transform = scipy.ndimage.distance_transform_edt(1-grid)

		return distance_transform, north_offset, east_offset

	def _local_to_grid(self, state: State):
		north_grid = int(state.local_position.north - self._north_offset)
		east_grid = int(state.local_position.east - self._east_offset)
		return north_grid, east_grid


	def search(self):
		start_pos = self._start_grid
		goal_pos = self._goal_grid
		visited = set()
		goal_is_found = False
		branch = {}
		queue = PriorityQueue()

		queue.put((0.0, start_pos))
		while not queue.empty():

			current_pos = queue.get()[1]
			print(current_pos)

			if current_pos == goal_pos:
				goal_is_found = True
				print("The goal has been found.")
				break

			for neighbor_and_cost in self._get_neighbors(current_pos):
				neighbor = neighbor_and_cost[0]
				if neighbor not in visited:
					g_score = neighbor_and_cost[1]
					h_score = np.linalg.norm(np.array([neighbor[0], neighbor[1]])- np.array(goal_pos))
					f_score = g_score + h_score
					queue.put((f_score, neighbor))
					branch[neighbor] = (current_pos, g_score)
					visited.add(neighbor)
		if goal_is_found:
			cost = 0.0
			current_pos = goal_pos
			path = [current_pos]
			while current_pos != start_pos:
				cost += branch[current_pos][1]
				path.append(branch[current_pos][0])
				current_pos = branch[current_pos][0]
			path.append(start_pos)
			path.reverse()
			self._path = path
			print(("The cost is", cost))




	def _get_neighbors(self, current_pos):
		valid_positions_and_costs = []
		for action in Action:
			new_pos = (current_pos[0] + action.delta[0], current_pos[1] + action.delta[1])
			if new_pos[0] < 0 or new_pos[0] >= self._north_size or new_pos[1] < 0 or new_pos[1] >= self._east_size:
				continue 
			if self._grid[new_pos[0], new_pos[1]] > 5.0:
				valid_positions_and_costs.append((new_pos, action.cost))
		return valid_positions_and_costs

	def visualize(self):
		
		plt.imshow(self._grid, origin="lower", cmap="jet")
		plt.colorbar()
		plt.scatter(self._start_grid[1], self._start_grid[0], label="Start", color="lime")
		plt.scatter(self._goal_grid[1], self._goal_grid[0], label="Goal",color="lime")
		plt.plot([pos[1] for pos in self._path], [pos[0] for pos in self._path], color="orange", label="Path")
		plt.title('Distance transform of the environment ')

		plt.legend()
		plt.show()

class Action(Enum):
	NORTH = (1, 0, 1)
	EAST = (0, 1, 1)
	SOUTH = (-1, 0, 1)
	WEST = (0, -1, 1)
	NORTHEAST = (1, 1, np.sqrt(2))
	NORTHWEST = (1, -1, np.sqrt(2))
	SOUTHEAST = (-1, 1, np.sqrt(2))
	SOUTHWEST = (-1, -1, np.sqrt(2))

	@property
	def delta(self):
		return self.value[0:2]
	@property
	def cost(self):
		return self.value[2]
	

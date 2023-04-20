from geodetic_position import GeodeticPosition
from halfsize import HalfSize 
from local_position import LocalPosition
import numpy as np
from obstacle import Obstacle
from sklearn.neighbors import KDTree
from typing import List


class ObstacleCollection:
	def __init__(self, obstacle_array: np.ndarray, geodetic_home: GeodeticPosition):
		self._list = self._build_list(obstacle_array)
		self._tree = self._build_tree()
		self._safety = self._determine_safety()

	def _determine_safety(self) -> float:
		"""Determine the largest hypot of obstacles in the list of obstacles"""
		safety = 0
		for obstacle in self._list:
			hypotenuse = np.hypot(obstacle.halfsize.north, obstacle.halfsize.east)
			if hypotenuse > safety:
				safety = hypotenuse

		# For added safety, return a safety distance equal to twice the longest obstacle
		# hypotenuse, rather than retur a safety distance that is merely the longest
		# obstacle hypotenuse. 
		
		return 2*safety

	def _build_list(self, obstacle_array: np.ndarray) -> List[Obstacle]:
		obstacle_list = []
		for row_index in range(obstacle_array.shape[0]):
			north, east, down, north_halfsize, east_halfsize, down_halfsize = obstacle_array[row_index, :]
			local_position = LocalPosition(north, east, down)
			halfsize = HalfSize(north_halfsize, east_halfsize, down_halfsize)
			obstacle_list.append(Obstacle(local_position, halfsize))

		return obstacle_list
	
	def _build_tree(self) -> KDTree:
		"""Returns a KDTree of obstacle ground-center positions"""
		ground_center_coords = []
		for obstacle in self._list:
			ground_center_coord = np.array([obstacle.local_position.north, obstacle.local_position.east])
			ground_center_coords.append(ground_center_coord)
		ground_center_coords = np.array(ground_center_coords)
		
		tree = KDTree(ground_center_coords)

		return tree

	def insert_obstacle_into_collection(self, new_obstacle: Obstacle) -> None:
		self._list.append(new_obstacle)
		self._tree = self._build_tree()

	@property
	def list(self):
		return self._list

	@property
	def tree(self):
		return self._tree

	@property
	def safety(self):
		return self._safety
import numpy as np
from sklearn.neighbors import KDTree

class StateCollection:
	def __init__(self, state_list):
		self._list = state_list
		self._tree = self._build_tree()

	def _build_tree(self):
		local_coords = []
		for state in self._list:
			north = state.local_position.north
			east = state.local_position.east
			down = state.local_position.down
			local_coord = np.array([north, east, down]) 
			local_coords.append(local_coord)
		tree = KDTree(local_coords)

		return tree

	@property
	def list(self):
		return self._list

	@property
	def tree(self):
		return self._tree


import numpy as np
from sklearn.neighbors import KDTree

class StateCollection:
	def __init__(self, start_state, goal_state, middle_states=None):
		self._start_state = start_state
		self._middle_states = middle_states
		self._goal_state = goal_state
		self._list = [start_state] + middle_states + [goal_state]
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

	@property
	def start_state(self):
		return self._start

	@property
	def middle_states(self):
		return self._middle_states

	@property
	def goal_state(self):
		return self._goal_state

class PotentialStateCollection(StateCollection):
	def __init__(self):
		super().__init__(start_state, goal_state, middle_states)
		pass



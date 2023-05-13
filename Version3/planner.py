from bounds import Bounds
from environment import Environment
from local_position import LocalPosition
import matplotlib.pyplot as plt
import numpy as np
from start_goal_pair import StartGoalPair
from state import State
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
		self._start_state = start_state
		self._goal_state = goal_state
		self._environment = environment
		self._state_space = None # This will be a state collection
		self._cost = None

	@property
	def state_space(self):
		return self._state_space

	@property
	def start_state(self):
		return self._start_state

	@property
	def goal_state(self):
		return self._goal_state

	@property
	def cost(self):
		return self._cost


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
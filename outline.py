import csv
import numpy as np
import utm
from sklearn.neighbors import KDTree
import random 
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.patches import Polygon
import networkx as nx
import itertools
import pdb

"""
Driver code

Initialize environment from file

Set geo goal using environment method


"""

class PathPlanning(Drone):
	#upon recieving a position update:
	#1. create a start,goal state pair object from the current and goal geo pos
	#2. create a state sampler object with the start goal pair and the choice of algo and the desired sample size
	#3. search the state samples with an input env to output a sequence of states 
	#4. command the drone to fly to the first waypoint in the state sequence
 
class Environment:
	def __init__(self, filename):
		self._filename = filename
		self._geo_home = self._read_geo_home()
		self._geo_goal = None
		self._obstacles = self._read_obstacle_data()
		self._obstacle_tree = self._build_obstacle_tree()
	
	def _read_geo_home(self):
		#open colliders file
		#parse the text
		#return GeodeticPosition(lon, lat, alt)

	def _read_raw_obstacle_data(self):
		#open colliders
		#return np.loadtxt
		#self._construct_obstacles
		##for each row, LocalPosition(), HalfSize()
		##append (LocalPosition, HalfSize) to self.obstacles list
		#return obstacle list

	def _build_obstacle_tree(self):
		#for each obstacle in the environment
		##store the north, east coordinate in a KDtree
		#return tree

	def collides(self, state):
		# given a state object return true if state collides with obstacle, return false otherwise.

	def add_obstacle(self, obstacle):
		#append object to obstacle list
		#rebuild obstacle kd tree

	def set_goal(self, geodetic_obj):
		self._goal = State(geodetic_obj)

	def set_home(self, geodetic_obj):
		self._home = State(geodetic_obj)


	@geo_home.setter
	def geo_home(self, lon, lat, alt):
		self._geo_home = GeodeticPosition(lon, lat, alt)

	@geo_goal.setter
	def geo_goal(self, lon, lat, alt):
		self._geo_goal = GeodeticPosition(lon, lat, alt)

class Obstacle:
	def __init__(self, local_position_obj, half_size_obj):
		self._local_position = None
		self._half_size = None
		self._safety = None

	def _determine_safety(self):
		return max(self._half_size.north, self._half_size.east)

class LocalPosition:
	def __init__(self, north, east, down):
		self._north = north
		self._east = east
		self._down = down

class HalfSize:
	def __init__(self, north_half_size, east_half_size, down_half_size):
		self._north = north_half_size
		self._east = east_half_size
		self._down = down_half_size

class GeodeticPosition:
	def __init__(self, lon, lat, alt):
		self._lon = lon
		self._lat = lat
		self._alt = alt

class State:
	def __init__(environment, local_or_geodetic_pos_obj, heading=0):
		# if the type of local or geo pos obj is local
		## assign .local to local; assign .geo using local_to_geo
		#if the type of local or geo pos obj is geo
		## assign geo pos obj to .geo; assign .local using global_to_local
		self._local = None
		self._global = None
		self._cost = None
	
	def _geo_to_local(self):
		pass
	def _local_to_geo(self):
		pass
	def _determine_cost(self):
		# set k1, k2
		# determine distance to closest obstacle in environment
		# determine distance 

class StartGoalPair:
	def __init__(self, start_state, goal_state):
		self._start_state = start_state
		self._goal_state = goal_state

class StateMaker:
	def __init__(self, start_goal_pair, n, algo):
		self._start_goal_pair = start_goal_pair
		self._n = n
		self._algo = algo

		if algo == 'local':
			next_state = self._local_algo()
		elif algo == 'global':
			next_state = self._global_algo()

	def _local_algo(self, environment):
		# Integrate cost of traveling from start to goal, and set that equal to default cost -> run sum_cost()
		# cost = self._start_goal_pair.start_state.cost + self._start_goal_pair.goal_state.cost
		# create an ellipsoid object between start and goal: ellipsoid = Ellipsoid(start_goal_pair, self.environment)
		# sample N states within ellipse that also don't collide with env: if not self.environment.collides(state) and ellipse.contains(state)
		# run a* search to determine the lowest cost path. self.astar() returns a sequence of states and the cost
		# if the cost of the lowest cost path beats the default cost then use it,
		## otherwise, don't use it and keep the cost the same as the default cost
		## return the first state after the start state in the state sequence from a star. Send that state to the drone.
		## have the drone command its position to the local position of that state. 

		
class Ellipsoid:
	def __init__(self, start_goal_pair, environment):

		self._environment = environment

		self._start_goal_pair = start_goal_pair

		self._f1 = np.array([start_goal_pair.start_state.local.north, 
							 start_goal_pair.start_state.local.east, 
							 start_goal_pair.start_state.local.down])

		self._f2 = np.array([start_goal_pair.goal_state.local.north, 
							 start_goal_pair.goal_state.local.east, 
							 start_goal_pair.goal_state.local.down])

		self._orientation = self._f2 - self._f1
		self._majoraxis = np.linalg.norm(self.f2 - self.f1)
		
		self._direction = self._orientation / self._majoraxis
		self._center = self._f1 + 1/2 * self._orientation
		self._h = self._center[0]
		self._k = self._center[1]
		self._l = self._center[2]
		self._a = 1/2 * self._majoraxis
		self._b = self._a / 2
		self.c = self._a / 2

	def contains(state):
		x = state.local.north
		y = state.local.east
		z = state.local.down
		a = self._a
		b = self._b
		c = self._c
		h = self._h
		k = self._k
		l = self._l 
		return (x - h)^2 / a^2 + (y - k)^2 / b^2 + (z - l)^2 / c^2 < 1










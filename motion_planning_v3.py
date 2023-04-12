import csv
from enum import Enum, auto
import msgpack
import numpy as np
import pdb
import time
from typing import Tuple, List, Dict
from udacidrone import Drone 
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID

"""
Utility functions
"""

"""
Flight states
"""
class States(Enum): 
	MANUAL = auto()
	ARMING = auto()
	TAKEOFF = auto()
	WAYPOINT = auto()
	LANDING = auto()
	DISARMING = auto()
	PLANNING = auto()


"""
Motion planning drone
"""
class MotionPlanning(Drone):
	"""
	# Initialize the drone
	# Position callback
	# Plan Path
		# Run A Star search on the search space from current location on grid to a target location within the prediction horizon.
	"""

class ObstacleData:

	def __init__(self, obstacle_file, path_params_file):
		self.north_bounds = None
		self.east_bounds = None
		self.alt_bounds = None
		self.grid = None
		self.geometry = None

	def read_obstacle_data(self, filename):
		# Northing bounds
		# Easting bounds
		# Altitude bounds
		# Grid
		# Shapely Polygon list of obstacle rectangles
		# Shapely Point list of obstacle center points
		# List of obstacle heights
		pass

class PathParams:

	def __init__(self, obstacle_file, path_params_file):
		self.home_geodetic = None
		self.goal_geodetic = None
		self.discretization = None

	def read_path_params(filename, filename2):
		# Destination longitude
		# Destination latitude
		# Destination altitude
		# Global home longitude
		# Global home latitude
		# Global home altitude
		# Type of search space
		pass

class SearchSpace:

	def __init__(self, obstacle_data, path_params):
		self.home_grid = None
		self.goal_grid = None
		self.search_space = None

	def build_elevation_map(self):
		print("Building a 2.5d grid representation of the environment...")
		pass

	def extract_obstacle_geometry(self):
		print("Extracting obstacle geometry...")
		pass

	def build_search_space(path_params, elevation_map):
		print("Building a search space...")
		# Build the right kind of search space.
		pass

	def build_2d_grid(elevation_map):
		print("Building a 2d grid...")
		pass

	def build_voronoi_graph(elevation_map):
		print("Building a Voronoi graph...")
		# Build a Voronoi diagram
		# Connect valid ridges of diagram to make a graph
		pass

	def build_prm_graph(elevation_map):
		print("Building a probabilistic roadmap (PRM) graph...")
		pass

	def build_rrt_graph(elevation_map):
		print("Building a rapidly-exploring random tree (RRT)...")
		pass

	def build_potential_field(elevation_map):
		print("Building a potential field...")
		pass

class Search:

	def __init__(self, search_space, path_params, obstacle_data):
		self.grid_start = None
		self.grid_goal = None
		self.search_space = None
		self.path_params = None
		self.obstacle_data = None

	def heuristic(self, current_grid, goal_grid):
		"""Computes the Euclidean distance between two positions on the grid"""
		pass

	def search_grid(self, search_space, grid_start, grid_goal, heuristic):
		"""Runs A* search on a grid, returning a path of waypoints in the local frame"""
		print("Searching the grid...")
		pass

	def search_graph(self, search_space, grid_start, grid_goal, heuristic):
		"""Runs A* search on a graph, returning a path of waypoints in the local frame"""
		print("Searching the graph...")
		pass

	def search_potential_field(self, search_space, grid_start, grid_goal):
		"""Runs A* search on a potential field, returning a path of waypoints in the local frame"""
		print("Searching the potential field...")
		pass

	def grid_to_local(self, grid_position):
		"""Converts a grid-frame coordinate into a local-frame coordinate"""
		pass

	def local_to_grid(self, local_position):
		"""Converts a local-frame coordinate into a grid-frame coordinate"""
		pass

class Visualize:
	
	def __init__(self, obstacle_data, path_params, search_space):
		self.obstacle_data = obstacle_data
		self.path_params = path_params
		self.search_space = search_space

	def visualize(self):
		pass

if __name__ == "__main__":

	# File names
	obstacle_file = "colliders.csv"
	path_params_file = "path_params.json"
	
	# Read obstacle data
	obstacle_data = ObstacleData(obstacle_file, path_params_file)

	# Read path paramaters
	path_params = PathParams(obstacle_file, path_params_file)

	# Construct a search space
	search_space = SearchSpace(obstacle_data, path_params)

	# Plot the search space
	search_space_plot = Visualize(search_space)
	search_space_plot.visualize()

	# Establish a connection with the quadcopter


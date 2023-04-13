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

class Battery:
	def __init__(self, battery_params_file):
		self.capacity = self.read_capacity(battery_params_file)
		self.distance_factor = self.read_distance_factor(battery_params_file)
		self.velocity_factor = self.read_veloicty_factor
		self.charge = self.capacity
		self.last_update_timestamp = None

	def read_capacity(self, battery_params_file):
		"""Amp hours"""
		return capacity 

	def read_distance_factor(self, battery_params_file):
		"""Amp hours per meter traveled"""
		return distance_factor

	def read_time_factor(self, battery_params_file):
		"""Amp hours per second"""
		return time_factor

	def charge(self):
		self.charge = self.capacity

	def lose_charge(self, meters_traveled, current_velocity, current_timestamp):
		self.charge -=  self.distance_factor * distance + self.time_factor * seconds_elapsed
		self.last_update_timestamp = current_timestamp

class States(Enum): 

	MANUAL = auto()
	ARMING = auto()
	TAKEOFF = auto()
	WAYPOINT = auto()
	LANDING = auto()
	DISARMING = auto()
	PLANNING = auto()

class MotionPlanning(Drone):
	"""
	# Initialize the drone
	# Position callback
	# Plan Path
		# Run A Star search on the search space from current location on grid to a target location within the prediction horizon.
	"""

class ObstacleData:

	def __init__(self, obstacle_file):
		"""
		obstacle_file:
			all_geometry_data = numpy array of obstacle geometry
			north_bounds = (obstacle_north_min, obstacle_north_max)
			east_bounds = (obstacle_east_min, obstacle_east_max)
			polygons
			center_points
			heights


		"""
		self.home_geodetic = self.read_home_geodetic(obstacle_file)
		self.all_geometry_data = self.extract_geometry(obstacle_file)
		self.north_bounds = self.extract_north_bounds(self.all_geometry_data)
		self.east_bounds = self.extract_east_bounds(self.all_geometry_data)
		self.polygons = None 
		self.center_points = None
		self.heights = None

	def read_home_geodetic(self, obstacle_file):
		
		print("Reading home geodetic from file...")

		return home_geodetic

	def extract_geometry(self, obstacle_file):

		print("Extracting obstacle geometry from file...")

		return all_geometry_data

	def extract_north_bounds(self, all_geometry_data):
		
		print("Extracting obstacle north bounds...")
		
		return north_bounds

	def extract_east_bounds(self, all_geometry_data):
		
		print("Extracting obstacle east bounds...")

		return east_bounds

	def extract_polygons(self, all_geometry_data):
		
		print("Extracting obstacle polygons...")

		return polygons

	def extract_center_points(self, all_geometry_data):

		print("Extracting obstacle center points...")

		return center_points

	def extract_heights(self, all_geometry_data):

		print("Extracting obstacle heights...")

		return heights 

class PathParams:

	def __init__(self, path_params_file):

		self.goal_geodetic = self.read_goal_geodetic(path_params_file)
		self.discretization = self.read_discretization(path_params_file)
		self.safety = self.read_safety_distance(path_params_file)

	def read_goal_geodetic(self, path_params_file):
		
		print("Reading goal geodetic from file...")

		return goal_geodetic

	def read_discretization(self, path_params_file):
		
		print("Reading discretization type from file...")

		return discretization

	def read_safety_distance(self, path_params_file):

		print("Reading safety distance from file...")

		return safety

class Discretization(Enum):

	GRID = auto()
	VORONOI = auto()
	PRM = auto()
	RRT = auto()
	POTENTIAL = auto()

	def __str__(self):

		if self == self.GRID:
			return '2d grid'
		if self == self.VORONOI:
			return 'Voronoi graph'
		if self == self.PRM:
			return 'Probabilistic roadmap (PRM)'
		if self == self.RRT:
			return 'Rapidly-exploring random tree (RRT)'
		if self == self.POTENTIAL:
			return 'Potential field'

class SearchSpace:

	def __init__(self, path_params, obstacle_data):
		
		self.start_geodetic = None
		self.start_local = None
		self.start_grid = None
		self.goal_geodetic = None
		self.goal_local = None
		self.goal_grid = None

		if path_params.discretization == Discretization.GRID:
			self.search_space = self.build_2d_grid(obstacle_data, path_params.)
		elif path_params.discretization == Discretization.VORONOI:
			self.search_space = self.build_voronoi_graph(obstacle_data)
		elif path_params.discretization == Discretization.PRM:
			self.search_space = self.build_prm_graph(obstacle_data)
		elif path_params.discretization == Discretization.RRT:
			self.search_space = self.build_rrt_graph(obstacle_data)
		elif path_params.discretization == Discretization.POTENTIAL:
			self.search_space = self.build_potential_field(obstacle_data)


	def set_start(self, start_geodetic):

		print(f"Setting start geodetic to {start_geodetic}")
		
		self.start_geodetic = start_geodetic
		self.start_local = self.global_to_local()
		self.start_grid = self.local_to_grid()

	def set_goal(self, goal_geodetic):

		print(f"Setting goal geodetic to {goal_geodetic}")

		self.goal_geodetic = goal_geodetic
		self.goal_local = self.global_to_local(goal_geodetic, home_geodetic)
		self.goal_grid = self.local_to_grid(self.goal_local)

	def global_to_local(geodetic_position, home_geodetic):

		return local_position

	def local_to_grid(self, local_position):

		print("Converting a local position to a grid position...")

		return grid_position


	def build_2d_grid(self, obstacle_data, altitude):
		
		print("Building a 2d grid search space from obstacle data...")

		return grid_2d
		

	def build_voronoi_graph(self, obstacle_data):
		
		print("Building a Voronoi graph search space from obstacle data...")
		
		# Build a Voronoi diagram
		
		# Connect valid ridges of diagram to make a graph

		return voronoi_graph
		

	def build_prm_graph(self, obstacle_data):
		
		print("Building a probabilistic roadmap (PRM) graph search space from obstacle data...")

		return prm_graph
		

	def build_rrt_graph(self, obstacle_data):
		
		print("Building a rapidly-exploring random tree (RRT) search space from obstacle data...")

		return rrt_graph

	def build_potential_field(self, obstacle_data):

		print("Building a potential field search space from obstacle data...")

		return potential_field

	def visualize(self):
		
		print("Visualizing the path...")

class WaypointFinder:

	def __init__(self, search_space, path_params):

		self.search_space = search_space
		self.path_params = path_params

	def search(self):

		if path_params.discretization == Discretization.GRID:
			waypoints = self.astar_grid(search_space)
		elif path_params.discretization == Discretization.VORONOI:
			waypoints = self.astar_graph(search_space)
		elif path_params.discretization == Discretization.PRM:
			waypoints = self.astar_graph(search_space)
		elif path_params.discretization == Discretization.RRT:
			waypoints = self.astar_graph(search_space)
		elif path_params.discretization == Discretization.POTENTIAL:
			waypoints = self.gradient_descent(search_space)

		return waypoints

	def euclidean_distance(self, gridcell_1, gridcell_2):

		return distance
	
	def astar_graph(self, search_space):

		print("Searching the graph for a path with A*...")

		return waypoints

	def astar_grid(self, search_space):
		
		print("Searching the grid for a path with A*")

		return waypoints

	def gradient_descent(self, search_space):
		
		print("Searching the potential field for a path using gradient descent...")

		return waypoints

	def path_to_waypoints(self, path):

		print("Converting the grid path into a sequence of waypoints...")

		return waypoints


if __name__ == "__main__":

	# File names
	obstacle_file = "colliders.csv"
	path_params_file = "path_params.json"
	
	# Read obstacle data
	obstacle_data = ObstacleData(obstacle_file)

	# Read path parameters
	path_params = PathParams(path_params_file)

	# Construct a search space
	search_space = SearchSpace(obstacle_data, path_params)

	# .. #
	# In the plan_path method of MotionPlanning, update search_space's start and goal. 
	# search_space.set_start(current_geodetic->local)
	# search_space.set_goal(next_waypoint)

	# Create a waypoint_finder object
	# waypoint_finder = WaypointFinder(search_space)
	# waypoints = waypoint_finder.search()

	

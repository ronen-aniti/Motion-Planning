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

read_obstacle_data(filename):
	# Northing bounds
	# Easting bounds
	# Altitude bounds
	# Grid
	# Shapely Polygon list of obstacle rectangles
	# Shapely Point list of obstacle center points
	# List of obstacle heights
	pass

read_path_params(filename, filename2):
	# Destination longitude
	# Destination latitude
	# Destination altitude
	# Global home longitude
	# Global home latitude
	# Global home altitude
	# Type of search space
	pass

build_search_space(path_params, elevation_map):
	print("Building a search space...")
	# Build the right kind of search space.
	pass

build_2d_grid(elevation_map):
	print("Building a 2d grid...")
	pass

build_voronoi_graph(elevation_map):
	print("Building a Voronoi graph...")
	# Build a Voronoi diagram
	# Connect valid ridges of diagram to make a graph
	pass

build_prm_graph(elevation_map):
	print("Building a probabilistic roadmap (PRM) graph...")
	pass

build_rrt_graph(elevation_map):
	print("Building a rapidly-exploring random tree (RRT)...")
	pass

build_potential_field(elevation_map):
	print("Building a potential field...")
	pass

search_grid(grid):
	print("Searching the grid...")
	pass

search_graph(grid):
	print("Searching the graph...")
	pass


if __name__ == "__main__":

	# Read the path configuration file
	path_params = read_path_params('path_params.json', 'colliders.csv')

	# Build a 2.5d map (elevation map) of the drone's environment
	obstacles = read_obstacle_data('colliders.csv')

	# Build 2D Grid, Voronoi graph, PRM graph, RRT graph, or potential field (grid) of the entire environment
	search_space = build_search_space(path_params, elevation_map)

	# Establish a connection with the quadcopter


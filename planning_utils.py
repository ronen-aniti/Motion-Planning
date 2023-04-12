import numpy as np
from sklearn.neighbors import KDTree
from scipy.spatial import Voronoi, voronoi_plot_2d
import networkx as nx
from bresenham import bresenham
from scipy.spatial import distance
from shapely.geometry import Polygon, Point
import matplotlib.pyplot as plt
import csv
import utm
from queue import PriorityQueue
from typing import Tuple, List, Dict, Callable, Set
import json
from enum import Enum, auto
import pdb

"""
Coordinate frames:

local_position = NED delta from global home (0,0,0)
ned_boundaries = base NED coordinates
obstacle center distance - ned_boundary = grid center value 
"""

class ElevationMap():
	def __init__(self, filename: str):
		self.elevation_map, self.ned_boundaries, self.map_size, self.map_data = self.build_map_and_take_measurements(filename)
		
	def build_map_and_take_measurements(self, filename):
		SAFETY_DISTANCE = 7.0

		# Convert CSV obstacle data to numpy array
		map_data = np.loadtxt(filename, delimiter=',', skiprows=2)

		# Calculate NED boundaries and map size
		ned_boundaries, map_size = calculate_ned_boundaries_and_map_size(map_data, SAFETY_DISTANCE)

		# Initialize a grid of zeros
		elevation_map = np.zeros((map_size[0], map_size[1]))

		# Build a 2.5D grid representation of the drone's environment
		for i in range(map_data.shape[0]):
			north, east, down, d_north, d_east, d_down = map_data[i, :]
			height = down + d_down
			obstacle_boundaries = [
				int(round(north - ned_boundaries[0] - d_north - SAFETY_DISTANCE)),
				int(round(north - ned_boundaries[0] + d_north + SAFETY_DISTANCE)),
				int(round(east - ned_boundaries[2] - d_east - SAFETY_DISTANCE)),
				int(round(east - ned_boundaries[2] + d_east + SAFETY_DISTANCE))
			]
			elevation_map[obstacle_boundaries[0]:obstacle_boundaries[1] + 1, obstacle_boundaries[2]:obstacle_boundaries[3] + 1] = height - ned_boundaries[4] + SAFETY_DISTANCE

		return elevation_map, ned_boundaries, map_size, map_data

class PlanningScheme(Enum):
	GRID_2D = auto()
	VORONOI = auto()
	PRM = auto()
	RRT = auto()
	POTENTIAL_FIELD = auto()

	def __str__(self):
		if self == self.GRID_2D:
			return 'Grid 2d'
		if self == self.VORONOI:
			return 'Voronoi graph'
		if self == self.PRM:
			return 'Probabilistic roadmap (PRM)'
		if self == self.RRT:
			return 'Rapidly-exploring random tree (RRT)'
		if self == self.POTENTIAL_FIELD:
			return 'Potential Field'

def build_a_connected_voronoi_graph(elevation_map: np.ndarray, voronoi_diagram: Voronoi, altitude: float):
	voronoi_graph_edges = []
	for ridge_vertex in voronoi_diagram.ridge_vertices:
		start_point_ridge_vertex_index = ridge_vertex[0] # Start point of ridge index
		end_point_ridge_vertex_index = ridge_vertex[1] # End point of ridge index
		start_point_of_candidate_edge = voronoi_diagram.vertices[start_point_ridge_vertex_index]
		end_point_of_candidate_edge = voronoi_diagram.vertices[end_point_ridge_vertex_index]
		start_point_northing_on_grid = int(round(start_point_of_candidate_edge[0]))
		start_point_easting_on_grid = int(round(start_point_of_candidate_edge[1]))
		end_point_northing_on_grid = int(round(end_point_of_candidate_edge[0]))
		end_point_easting_on_grid = int(round(end_point_of_candidate_edge[1]))
		grid_cells_between_points = bresenham(start_point_northing_on_grid,
												start_point_easting_on_grid,
												end_point_northing_on_grid,
												end_point_easting_on_grid)
		collides = False
		
		northing_max = elevation_map.shape[0]
		easting_max = elevation_map.shape[1]

		for grid_cell in grid_cells_between_points:
			grid_cell_northing = grid_cell[0]
			grid_cell_easting = grid_cell[1]
			# Check if grid cell in question is off-grid
			if (grid_cell_northing < 0 or grid_cell_northing >= northing_max or 
					grid_cell_easting < 0 or grid_cell_easting >= easting_max):
				collides = True
				break
			# Check if grid cell in question is inside an obstacle
			if elevation_map[grid_cell_northing][grid_cell_easting] > altitude:
				collides = True
				break
		if not collides:
			start_point_of_edge = (int(round(start_point_of_candidate_edge[0])), 
									int(round(start_point_of_candidate_edge[1])))
			end_point_of_edge = (int(round(end_point_of_candidate_edge[0])), 
									int(round(end_point_of_candidate_edge[1])))

			voronoi_graph_edges.append((start_point_of_edge, end_point_of_edge))

	# Stepping through each edge
		
	pdb.set_trace()

	return voronoi_graph_edges 






def extract_obstacle_geometry(map_data: np.ndarray, ned_boundaries: List[int]) -> List[Tuple[Polygon, Point, float]]:
	"""Return a set containing a tuple for each obstacle that contains a Shapely Polygon object, a Shapely
	Point object, and a float value indicating the obstacle's height"""
	
	SAFETY_DISTANCE = 10.0

	obstacle_geometry = []
	for row_number in range(map_data.shape[0]):
		north, east, down, d_north, d_east, d_down = map_data[row_number, :]
		obstacle_center_on_grid = Point(np.array([int(round(north - ned_boundaries[0])), int(round(east - ned_boundaries[2]))]))
		obstacle_boundaries_on_grid = [
			int(round(north - ned_boundaries[0] - d_north - SAFETY_DISTANCE)),
			int(round(north - ned_boundaries[0] + d_north + SAFETY_DISTANCE)),
			int(round(east - ned_boundaries[2] - d_east - SAFETY_DISTANCE)),
			int(round(east - ned_boundaries[2] + d_east + SAFETY_DISTANCE))
		]
		point_1 = np.array([obstacle_boundaries_on_grid[0], obstacle_boundaries_on_grid[2]])
		point_2 = np.array([obstacle_boundaries_on_grid[0], obstacle_boundaries_on_grid[3]])
		point_3 = np.array([obstacle_boundaries_on_grid[1], obstacle_boundaries_on_grid[3]])
		point_4 = np.array([obstacle_boundaries_on_grid[1], obstacle_boundaries_on_grid[2]])

		boundary_points_as_array = np.array([point_1, point_2, point_3, point_4])
		obstacle_polygon_on_grid = Polygon(boundary_points_as_array)
		obstacle_height = down + d_down - ned_boundaries[4] + SAFETY_DISTANCE
		obstacle_geometry.append((obstacle_polygon_on_grid, obstacle_center_on_grid, obstacle_height))

	return obstacle_geometry




def get_user_planning_scheme() -> PlanningScheme:
	"""Prompt the user to select a planning scheme"""

	print("Choose a planning scheme:")
	for scheme in PlanningScheme:
		print(f"{scheme}: {scheme.value}")
	choice = input("Enter the number corresponding to your choice: ")

	try:
		selected_scheme = PlanningScheme(int(choice))
	except ValueError:
		print("Invalid choice. Please try again.")
		return get_user_planning_scheme()

	return selected_scheme

def read_global_home(filename: str) -> Tuple[float, float, float]:
	"""
	Extracts and returns the geodetic global home coordinate from the obstacle file.

	Parameters
	----------
	filename : str
		The name of the obstacle file containing the global home coordinates.

	Returns
	-------
	Tuple[float, float, float]
		A tuple containing the global home coordinates as (longitude, latitude, altitude).
	"""
	
	# Open the file and create a CSV reader object
	with open(filename) as f:
		reader = csv.reader(f)

		# Read the first line of the file
		first_line = next(reader)

	# Split the first line to extract latitude and logitude 
	lat0_str = first_line[0].split('lat0 ')[1]
	lon0_str = first_line[1].split(' lon0 ')[1]

	# Convert latitude and longitude strings to float values
	lat0 = float(lat0_str)
	lon0 = float(lon0_str)

	# Return the global home coordinates as a tuple (lon, lat, alt)
	return (lon0, lat0, 0.0)

def create_voronoi_diagram(map_data: np.ndarray, ned_boundaries: List[int], map_size: List[int], altitude: float) -> Voronoi:
	SAFETY_DISTANCE = 10.0
	obstacle_center_coordinates = []
	for i in range(map_data.shape[0]):
		north, east, down, d_north, d_east, d_down = map_data[i, :]
		if down + d_down + SAFETY_DISTANCE > altitude:
			obstacle_center_coordinates.append((int(round(north - ned_boundaries[0])), int(round(east - ned_boundaries[2]))))

	# Construct a Voronoi diagram
	voronoi_diagram = Voronoi(obstacle_center_coordinates)

	return voronoi_diagram

def build_map_and_take_measurements(filename: str) -> Tuple[np.ndarray, List[int], List[int], np.array]:
	"""
	Returns a 2.5D map of the drone's environment and NED boundaries.

	Parameters
	----------
	filename : str
		The name of the obstacle file containing obstacle data.

	Returns
	-------
	Tuple[np.ndarray, List[int], List[int]]
		A tuple containing the following:
		- elevation_map (np.ndarray): A 2.5D map of the drone's environment.
		- ned_boundaries (List[int]): A list containing the NED boundaries [north_min, north_max, east_min, east_max, alt_min, alt_max].
		- map_size (List[int]): A list containing the size of the map [north_size, east_size, alt_size].
	"""
	
	SAFETY_DISTANCE = 7.0

	# Convert CSV obstacle data to numpy array
	map_data = np.loadtxt(filename, delimiter=',', skiprows=2)

	# Calculate NED boundaries and map size
	ned_boundaries, map_size = calculate_ned_boundaries_and_map_size(map_data, SAFETY_DISTANCE)

	# Initialize a grid of zeros
	elevation_map = np.zeros((map_size[0], map_size[1]))

	# Build a 2.5D grid representation of the drone's environment
	for i in range(map_data.shape[0]):
		north, east, down, d_north, d_east, d_down = map_data[i, :]
		height = down + d_down
		obstacle_boundaries = [
			int(round(north - ned_boundaries[0] - d_north - SAFETY_DISTANCE)),
			int(round(north - ned_boundaries[0] + d_north + SAFETY_DISTANCE)),
			int(round(east - ned_boundaries[2] - d_east - SAFETY_DISTANCE)),
			int(round(east - ned_boundaries[2] + d_east + SAFETY_DISTANCE))
		]
		elevation_map[obstacle_boundaries[0]:obstacle_boundaries[1] + 1, obstacle_boundaries[2]:obstacle_boundaries[3] + 1] = height - ned_boundaries[4] + SAFETY_DISTANCE


	return elevation_map, ned_boundaries, map_size, map_data

def calculate_ned_boundaries_and_map_size(map_data: np.ndarray, safety_distance: float) -> Tuple[List[int], List[int]]:
	"""
	Calculate and return the NED boundaries and map size based on the given map data and safety distance.

	Parameters
	----------
	map_data : np.ndarray
		The map data as a NumPy array containing obstacle information.
	safety_distance : float
		The safety distance to be considered when calculating NED boundaries.

	Returns
	-------
	Tuple[List[int], List[int]]
		A tuple containing the following:
		- ned_boundaries (List[int]): A list containing the NED boundaries [north_min, north_max, east_min, east_max, alt_min, alt_max].
		- map_size (List[int]): A list containing the size of the map [north_size, east_size, alt_size].
	"""

	# Calculate NED boundaries: North min, North max, East min, East max, Alt min, Alt max
	ned_boundaries = [
	int(round(np.floor(np.amin(map_data[:, 0] - map_data[:, 3])) - safety_distance)),
	int(round(np.ceil(np.amax(map_data[:, 0] + map_data[:, 3])) + safety_distance)),
	int(round(np.floor(np.amin(map_data[:, 1] - map_data[:, 4])) - safety_distance)),
	int(round(np.ceil(np.amax(map_data[:, 1] + map_data[:, 4])) + safety_distance)),
	0,
	int(round(np.ceil(np.amax(map_data[:, 2] + map_data[:, 5])) + safety_distance))
	]


	# Calculate the size of the map
	map_size = [
		ned_boundaries[1] - ned_boundaries[0],
		ned_boundaries[3] - ned_boundaries[2],
		ned_boundaries[5] - ned_boundaries[4]
	]

	return ned_boundaries, map_size 

def build_voronoi_graph(altitude: float):
	pass

def read_destinations(filename: str) -> List[Dict[str, float]]:
	"""
	Reads destination coordinates from a JSON file and returns them as a list of dictionaries.
	
	Parameters
	----------
	filename : str
		The name of the JSON file containing the destination coordinates.
	
	Returns
	-------
	list of dict
		A list of dictionaries with destination coordinates in the format:
		[
			{'lat': latitude, 'lon': longitude, 'alt': altitude},
			...
		]
	"""
	with open("destinations.json", "r") as infile:
		loaded_destinations = json.load(infile)

	return loaded_destinations

def global_to_local(global_position: np.ndarray, global_home: np.ndarray) -> np.ndarray:
	"""
	Convert global (latitude, longitude, altitude) position to a local (NED) position relative to the global home.
	
	Parameters
	----------
	global_position : numpy.ndarray
		A 1D numpy array containing the global position as [longitude, latitude, altitude].
	global_home : numpy.ndarray
		A 1D numpy array containing the global home position as [longitude, latitude, altitude].
	
	Returns
	-------
	numpy.ndarray
		A 1D numpy array representing the local position as [northing, easting, altitude] relative to the global home.
	"""

	# Get easting and northing of global home
	lon_home, lat_home, alt_home = global_home
	easting_home, northing_home, _, _ = utm.from_latlon(lat_home, lon_home)
	
	# Get easting and northing of global position
	lon_pos, lat_pos, alt_pos = global_position
	easting_pos, northing_pos, _, _ = utm.from_latlon(lat_pos, lon_pos)
	
	# Calculate local position as NED coordinates relative to global home
	local_position = np.array([
		northing_pos - northing_home,
		easting_pos - easting_home,
		alt_home - alt_pos
	])
	
	return local_position

def calculate_nearest_free_cell_in_2d(elevation_map: np.ndarray, northing_index: int, easting_index: int, altitude: float) -> Tuple[int, int]:
	free_cell_found = False
	search_radius = 1
	max_northing, max_easting = elevation_map.shape

	while not free_cell_found:
		for i in range(-search_radius, search_radius + 1):
			for j in range(-search_radius, search_radius + 1):
				new_northing = northing_index + i 
				new_easting = easting_index + j 
				# Check if the new indices are within the bounds of the elevation_map
				if 0 <= new_northing < max_northing and 0 <= new_easting < max_easting:
					if elevation_map[northing_index + i][easting_index + j] < altitude:
						return new_northing, new_easting
		search_radius += 1 

class Actions(Enum):
	NORTH = (1, 0, 1)
	EAST = (0, 1, 1)
	SOUTH = (-1, 0, 1)
	WEST = (0, -1, 1)
	NORTHEAST = (1, 1, np.sqrt(2))
	NORTHWEST = (1, -1, np.sqrt(2))
	SOUTHEAST = (-1, 1, np.sqrt(2))
	SOUTHWEST = (-1, -1, np.sqrt(2))

	@property 
	def cost(self):
		return self.value[2]

	@property
	def delta(self):
		return self.value[:2]

	@property
	def delta_north(self):
		return self.value[0]

	@property
	def delta_east(self):
		return self.value[1]

def path_to_waypoints(path: List[Tuple[int, int]], ned_boundaries: Tuple[float], goal_altitude: float) -> List[List[float]]:
	return [[northing_index + ned_boundaries[0], easting_index + ned_boundaries[2], goal_altitude, 0] for northing_index, easting_index in path]

def remove_collinear(path: List[Tuple[int, int]]):
	"""Removes the collinear elements from path"""
	i = 0
	while len(path) > i + 2:
		x1, y1 = path[i]
		x2, y2 = path[i + 1]
		x3, y3 = path[i + 2]
		array = np.array([[x1, y1, 1], [x2, y2, 1], [x3, y3, 1]])
		tolerance = 0.01
		collinear = np.linalg.det(array) <= tolerance
		#collinear = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2) == 0
		if collinear:
			del path[i + 1]
		else:
			i += 1

	return path

def valid_actions(elevation_map: np.ndarray, gridcell: Tuple[int, int], altitude: float) -> List[Actions]:
	"""Returns a list of Tuples containing a Tuple grid index and cost of traveling to a neighboring gridcell"""
	grid_northing, grid_easting =  gridcell

	actions = [Actions.NORTH, Actions.EAST, Actions.SOUTH, Actions.WEST, Actions.NORTHEAST, Actions.NORTHWEST, Actions.SOUTHEAST, Actions.SOUTHWEST]
	
	valid = []
	for action in actions:
		northing = grid_northing + action.delta_north
		easting = grid_easting + action.delta_east
		if 0 <= northing < elevation_map.shape[0] and 0 <= easting < elevation_map.shape[1]:
			if altitude > elevation_map[northing][easting]: 
				valid.append(action)

	return valid

def a_star(elevation_map: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int], goal_altitude: float, heuristic_function):
	"""Returns a path from start to goal, along with the cost"""
	
	path = []
	path_cost = 0.0
	queue = PriorityQueue()
	queue.put((0.0, start))
	visited = set(start)
	branch = {}
	found = False

	while not queue.empty():
		 
		_, current_node = queue.get()

		if current_node == start:
			current_cost = 0.0
		else:
			current_cost = branch[current_node][0]

		if current_node == goal:
			print("Found a path.")
			found = True
			break
		else:
			for action in valid_actions(elevation_map, current_node, goal_altitude):
				next_node = (current_node[0] + action.delta_north, current_node[1] + action.delta_east)
				branch_cost = current_cost + action.cost
				queue_cost = branch_cost + heuristic_function(next_node, goal)

				if next_node not in visited:
					visited.add(next_node)
					branch[next_node] = (branch_cost, current_node, action)
					queue.put((queue_cost, next_node))
	if found:
		n = goal 
		path_cost = branch[n][0]
		path.append(goal)
		while branch[n][1] != start:
			path.append(branch[n][1])
			n = branch[n][1]
		path.append(branch[n][1])
	else:
		print("Failed to find a path.")

	return path[::-1], path_cost

def astar_graph(graph, start, goal, h):
	pass
def astar_voxel(voxmap, start, goal, h):
	pass
def voxel_map(coordinate, r=40, dz=20):
	pass
def medial_axis(coordinate, r=40):
	pass
def voronoi_graph(coordinate, r=40):
	pass
def rrt(coordinate, r=40):
	pass
def potential_field(coordinate, r=40):
	pass

if __name__ == '__main__':
	print(Actions.NORTH.delta)
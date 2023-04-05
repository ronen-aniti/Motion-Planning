import numpy as np
import csv
import utm
from typing import Tuple, List, Dict, Callable
import json
import pdb

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

def build_map_and_take_measurements(filename: str) -> Tuple[np.ndarray, List[int], List[int]]:
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
	
	SAFETY_DISTANCE = 5.0

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
			int(north - ned_boundaries[0] - d_north - SAFETY_DISTANCE),
			int(north - ned_boundaries[0] + d_north + SAFETY_DISTANCE),
			int(east - ned_boundaries[2] - d_east - SAFETY_DISTANCE),
			int(east - ned_boundaries[2] + d_east + SAFETY_DISTANCE)
		]
		elevation_map[obstacle_boundaries[0]:obstacle_boundaries[1] + 1, obstacle_boundaries[2]:obstacle_boundaries[3] + 1] = height - ned_boundaries[4]

	return elevation_map, ned_boundaries, map_size

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
	int(np.floor(np.amin(map_data[:, 0] - map_data[:, 3])) - safety_distance),
	int(np.ceil(np.amax(map_data[:, 0] + map_data[:, 3])) + safety_distance),
	int(np.floor(np.amin(map_data[:, 1] - map_data[:, 4])) - safety_distance),
	int(np.ceil(np.amax(map_data[:, 1] + map_data[:, 4])) + safety_distance),
	0,
	int(np.ceil(np.amax(map_data[:, 2] + map_data[:, 5])) + safety_distance)
	]


	# Calculate the size of the map
	map_size = [
		ned_boundaries[1] - ned_boundaries[0],
		ned_boundaries[3] - ned_boundaries[2],
		ned_boundaries[5] - ned_boundaries[4]
	]

	return ned_boundaries, map_size 

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

def collides(elevation_map: np.ndarray, northing_index: int, easting_index: int, altitude: float) -> bool:
	"""
	Determines if given coordinate (norrthing, easting, down) is occupied.

	Parameters
	----------
	elevation_map : numpy.ndarray
		A 2D numpy array containing the 2.5D map of the drone's environment
	northing_index : int
		The northing measurement (meters) of the goal location
	easting_index : int
		The easting measurement (meters) of the goal location
	down : float
		The down measurement (meters) of the goal location 

	Returns
	-------
	bool
		A bool indicating whether or not the given NED coordinate is occupied

	"""
	return altitude <= elevation_map[northing_index, easting_index]

def calculate_nearest_free_cell_in_2d(elevation_map: np.ndarray, northing_index: int, easting_index: int, altitude: float) -> Tuple[int, int, float]:
	free_cell_found = False
	search_radius = 1

	while not free_cell_found:
		for i in range(-search_radius, search_radius + 1):
			for j in range(-search_radius, search_radius + 1):
				if not collides(elevation_map, northing_index + i, easting_index + j, altitude):
					return northing_index + i, easting_index + j, altitude
		search_radius += 1 

def euclidean_distance(point: np.ndarray, goal: np.ndarray) -> float:
	return np.linalg.norm(goal - point)

def astar(elevation_map: np.ndarray, start_northing: int, start_easting: int, goal_northing: int, goal_easting: int, heuristic_function: Callable[[np.ndarray, np.ndarray], float]):
	pass

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
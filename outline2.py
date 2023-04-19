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

# Driver code
filename = "colliders.csv"
reader = ObstacleFileReader(filename) # -> extract_global_home() -> State from GeodeticPosition,  
geodetic_home = reader.extract_geodetic_home() #-> GeodeticPosition 
obstacle_array = reader.extract_obstacles_as_array() #-> np.ndarray
obstacle_collection = ObstacleCollection(obstacle_array, geodetic_home) # .list List[Obstacle]; .tree KDtree
geodetic_goal = GeodeticPosition(-122.3123, 37.1231, 10) # modify the exact position floats
environment = Environment(geodetic_home, geodetic_goal, obstacle_collection)
global_start_goal_pairing = StartGoalPair(geodetic_home, geodetic_goal)
global_sampler = GlobalSampler(global_start_goal_pairing) #Global sampler object with global plan method
global_state_seqeunce = global_sampler.determine_global_state_sequence() # List[State]
waypoints = global_sampler.convert_state_sequence_to_waypoints(global_state_sequence) #List[float]
# Send the state sequence to the drone as waypoint list

# Upon position update...
## Potentially simulate adding an obstacle into the obstacle collection using obstacle_collection.insert_obstacle_into_list(new_obstacle)
longitude, latitude, altitude = -122.1111, 37.222, 9.827 # Get these from drone's GPS
geodetic_position = GeodeticPosition(longitude, latitude, altitude)
target_geodetic_position = global_state_sequence[target_state_index].geodetic_position # Increment target state sequence each time the drone's within deadband radius of target
local_start_goal_pairing = StartGoalPair(geodetic_position, target_geodetic_position)
local_sampler = LocalSampler(local_start_goal_pairing)
next_waypoint = local_sampler.determine_next_state().waypoint


class ObstacleFileReader:
	def __init__(self, filename: str):
		self._filename = filename
	def extract_geodetic_home(self) -> GeodeticPosition:
		pass
	def extract_obstacles_as_array(self) -> np.ndarray:
		pass

class ObstacleCollection:
	def __init__(self, obstacle_array: np.ndarray, geodetic_home: GeodeticPosition):
		self._list = self._build_list()
		self._tree = self._build_tree()
		self._safety = self._determine_safety()

	def _determine_safety(self) -> float:
		# Determine the largest hypot of obstacles in the list of obstacles
		pass
	def _build_list(self) -> List[Obstacle]:
		pass
	def _build_tree(self) -> KDTree:
		pass
	def _insert_obstacle_into_list(self, new_obstacle: Obstacle) -> None:
		pass
	def _rebuild_tree(self, new_obstacle_list: List[Obstacle]) -> None:
		pass

	@property
	def list(self):
		return self._list

	@property
	def tree(self):
		return self._tree

	@property
	def safety(self):
		return self._safety
	

class StartGoalPair:
	def __init__(self, start: GeodeticPosition, goal: GeodeticPosition):
		self._start = start
		self._goal = goal

	@property
	def start(self):
		return self._start	

	@property
	def goal(self):
		return self._goal

class GeodeticPosition:
	def __init__(self, longitude: float, latitude: float, altitude: float):
		self._longitude = longitude
		self._latitude = latitude
		self._altitude = altitude

	@property
	def longitude(self):
		return self._longitude

	@property
	def latitude(self):
		return self._latitude

	@property
	def altitude(self):
		return self._altitude

class LocalPosition:
	def __init__(self, north: float, east: float, down:float):
		self._north = north
		self._east = east
		self._down = down

	@property
	def north(self):
		return self._north
	
	@property
	def east(self):
		return self._east
	@property
	def down(self):
		return self._down



class Environment:
	def __init__(self, geodetic_home: GeodeticPosition, geodetic_goal: GeodeticPosition, obstacle_collection: ObstacleCollection):
		self._north_bounds = self._determine_north_bounds() # Build and return Bounds Object from obstacle_collection
		self._east_bounds = self._determine_east_bounds()
		self._down_bounds = self._determine_down_bounds()
		self._obstacle_collection = obstacle_collection
		self._geodetic_home = geodetic_home
		self._geodetic_goal = geodetic_goal

	def _determine_north_bounds(self) -> Bounds:
		pass

	def _determine_east_bounds(self) -> Bounds:
		pass

	def _determine_down_bounds(self) -> Bounds:
		pass

	def collides(self, state: State) -> bool:
		pass

	def add_obstacle(self, new_obstacle) -> None:
		pass

	def _update_bounds(self) -> None:
		#Should find the bounds based on the obstacle tree or list
		pass

	@property
	def north_bounds(self):
		return self._north_bounds

	@property
	def east_bounds(self):
		return self._east_bounds

	@property
	def down_bounds(self):
		return self._down_bounds

	@property
	def obstacle_collection(self):
		return self._obstacle_collection

	@property
	def geodetic_home(self):
		return self._geodetic_home

	@property
	def geodetic_goal(self):
		return self._geodetic_goal

class Bounds:
	def __init__(self, minimum, maximum):
		self._minimum = minimum
		self._maximum = maximum

	@property
	def minimum(self):
		return self._minimum

	@property
	def maximum(self):
		return self._maximum

class Obstacle:
	def __init__(self, local_position: LocalPosition, halfsize: HalfSize):
		self._local_position = local_position
		self._halfsize = halfsize
		self._safety = self._determine_safety()

	def _determine_safety(self) -> float:
		pass

	@property
	def local_position(self):
		return self._local_position

	@property
	def half_size(self):
		return self._half_size

	@property
	def safety(self):
		return self._safety

class HalfSize:
	def __init__(self, north, east, down):
		self._north = north
		self._east = east
		self._down = down

	@property
	def north(self):
		return self._north
	
	@property
	def east(self):
		return self._east

	@property
	def down(self):
		return self._down

class State:
	def __init__(self, geodetic_pairing: StartGoalPair, environment: Environment):
		self._geodetic_position = geodetic_pairing.goal
		self._local_position = LocalPosition(self._global_to_local(geodetic_pairing))
		self._heading = 0
		self._waypoint = [self._local_position.north, self._local_position.east, -1.0 * self._local_position.down, self._heading] #List[float]
		self._environment = environment
		self._ground_distance_to_nearest_obstacle = self._determine_distance_to_nearest_obstacle()
		self._ground_distance_to_goal = self._determine_distance_to_goal()

	def _global_to_local(self, geodetic_pairing: StartGoalPair) -> np.ndarray:
		pass

	def _determine_distance_to_nearest_obstacle(self) -> float:
		pass

	def _determine_distance_to_goal(self) -> float:
		pass

	@property
	def geodetic_position(self):
		return self._geodetic_position

	@property
	def local_position(self):
		return self._local_position

	@property
	def heading(self):
		return self._heading

	@property
	def waypoint(self):
		return self._waypoint

	@property
	def environment(self):
		return self._environment

	@property
	def ground_distance_to_nearest_obstacle(self):
		return self._ground_distance_to_nearest_obstacle

	@property
	def ground_distance_to_goal(self):
		return self._ground_distance_to_goal


class StateSampler:
	def __init__(self, pairing: StartGoalPair):
		self._start_goal_pair = pairing

	def convert_state_to_waypoint(state) -> np.ndarray:
		pass

	@property
	def start_goal_pair(self):
		return self._start_goal_pair

class LocalSampler(StateSampler):
	def __init__(self, pairing: StartGoalPair):
		super().__init__(pairing)

	def determine_next_state(self, environment: Environment, num_samples: int):
		#Conduct A* search through samples. The minimum cost path should be directly from start to goal. However,
		#if a new obstacle blocks that path, the A* search would plan around the block.
		#return next state to fly to using local algo

class GlobalSampler(StateSampler):
	def __init__(self, pairing: StartGoalPair):
		super().__init__(start_goal_pair)

	def determine_global_state_sequence(self, environment: Environment, num_samples: int) -> List[State]:
		#Implement A* search from start state to goal state given the environment and a sample size.
		#Use a uniform random distribution to generate samples that don't have any obstacles within the 
		#environment's obstacle collection's safety radius. Return the list of states that the drone should
		#plan to fly through. 
		pass 
		
	def convert_state_sequence_to_waypoints(global_state_sequence: List[State]) -> List[float]
		pass
	"""
	Ex. 
	local_sampler = StateSampler(local_start_goal_pair, 200, 'local')
	next_state = local_sampler.search(whole_environment)
	"""
	pass



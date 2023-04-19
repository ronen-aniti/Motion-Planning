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
	def __init__(self, battery_params_file, timestamp):
		self._capacity = self._read_capacity(battery_params_file)
		self._distance_factor = self._read_distance_factor(battery_params_file)
		self._time_factor = self._read_time_factor(battery_params_file)
		self._charge = self.capacity
		self._last_update_timestamp = timestamp

	def _read_capacity(self, battery_params_file):
		"""Amp hours"""
		return capacity 

	def _read_distance_factor(self, battery_params_file):
		"""Amp hours per meter traveled"""
		return distance_factor

	def _read_time_factor(self, battery_params_file):
		"""Amp hours per second"""
		return time_factor

	def recharge(self):
		print("Recharging battery...")
		self._charge = self._capacity
		print("The battery is now fully charged")

	def lose_charge(self, distance, current_timestamp):
		seconds_elapsed = current_timestamp - self._last_update_timestamp
		self._charge -=  self._distance_factor * distance + self._time_factor * seconds_elapsed
		self._last_update_timestamp = current_timestamp

	@property
	def charge(self):
		return self._charge

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

class States(Enum): 

	MANUAL = auto()
	ARMING = auto()
	TAKEOFF = auto()
	WAYPOINT = auto()
	LANDING = auto()
	DISARMING = auto()
	COARSE_PLANNING = auto()
	LOCAL_PLANNING = auto()

class MotionPlanning(Drone):
	"""
	# Initialize the drone
	# Position callback
	# Plan Path
		# Run A Star search on the search space from current location on grid to a target location within the prediction horizon.
	"""

	def __init__(self, connection, planner):
		super().__init__(connection)

		self.planner = planner

		self.target_local = np.array([0.0, 0.0, 0.0, 0.0])
		self.waypoints = []
		self.in_mission = True
		self.check_state = {}

		self.flight_state = States.MANUAL

		self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
		self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
		self.register_callback(MsgID.STATE, self.state_callback)


	def local_position_callback(self):
		if self.flight_state == States.TAKEOFF:
			if self.local_position[2] > 0.95 * self.search_space.goal_geodetic[2]:
				self.waypoint_transition()
		drone_speed = np.linalg.norm(self.local_velocity)
		deadband = 0.25 + 0.25 * drone_speed
		elif self.flight_state == States.WAYPOINT:
			if np.linalg.norm(self.local_position[0:2] - self.search_space.goal_local[0:2]) < deadband:
				self.landing_transition()
			if np.linalg.norm(self.local_position[0:2] - self.target_local[0:2]) < deadband:
				self.waypoint_transition()

	def velocity_callback(self):
		if self.flight_state == States.LANDING:
			if (self.global_position[2] - self.global_home[2]) < 0.1:
				if abs(self.local_position[2]) < 0.01:
					self.disarming_transition()

	def state_callback(self):
		if self.in_mission:
			if self.flight_state == States.MANUAL:
				self.arming_transition()
			elif self.flight_state == States.ARMING:
				if self.armed:
					self.plan_path()
			elif self.flight_state == States.PLANNING:
				self.takeoff_transition()
			elif self.flight_state == States.DISARMING:
				if ~self.armed & ~self.guided:
					self.manual_transition()
	
	def manual_transition(self):
		self.flight_state = States.MANUAL
		print("Manual transition")
		self.stop()
		self.in_mission = False

	def arming_transition(self): 
		self.flight_state = States.ARMING
		global_home = self.search_space.home_geodetic
		self.set_home_position(*global_home)
		print("Arming transition")
		self.take_control()
		self.arm()

	def takeoff_transition(self):
		self.flight_state = States.TAKEOFF
		print("Takeoff transition")
		self.takeoff(self.target_local[2])

	def waypoint_transition(self):
		self.flight_state = States.WAYPOINT
		print("Waypoint transition")
		self.cmd_position(self.target_local[0], self.target_local[1], self.target_local[2], self.target_local[3])

	def landing_transition(self):
		self.flight_state = States.LANDING
		print("Landing transition")
		self.land()

	def disarming_transition(self):
		self.flight_state = States.DISARMING
		print("Disarming transition")
		self.disarm()
		self.release_control()

	def plan_coarse_path(self, planner):

		self.flight_state = States.COARSE_PLANNING
		self.target_local = self.search_space.goal_local

		# Get current geodetic
		planner.search_space.set_start(self.current_global_position)
		planner.search_space.set_goal(self.search_space.goal_geodetic)
		waypoints = planner.search()



	def plan_local_path(self):
		pass

	def send_waypoints(self):
		print("Sending waypoints to simulator...")
		data = msgpack.dumps(self.waypoints)
		self.connection._master.write(data)

	def start(self):
		self.start_log("Logs", "NavLog.txt")
		print("Starting connection")
		super().start()
		self.stop_log()

class Obstacles:

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
		self._home_geodetic = self.read_home_geodetic(obstacle_file)
		self._all_geometry_data = self.extract_geometry(obstacle_file)
		self._north_bounds = self.extract_north_bounds(self.all_geometry_data)
		self._east_bounds = self.extract_east_bounds(self.all_geometry_data)
		self._polygons = None
		self._center_points = None
		self._heights = None

	def _read_home_geodetic(self, obstacle_file):
		
		print("Reading home geodetic from file...")

		return home_geodetic

	def _extract_geometry(self, obstacle_file):

		print("Extracting obstacle geometry from file...")

		return all_geometry_data

	def _extract_north_bounds(self, all_geometry_data):
		
		print("Extracting obstacle north bounds...")
		
		return north_bounds

	def _extract_east_bounds(self, all_geometry_data):
		
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

	@property
	def home_geodetic(self):
		return self._home_geodetic

	@property
	def all_geometry_data(self):
		return self._all_geometry_data

	@property
	def north_bounds(self):
		return self._north_bounds

	@property
	def east_bounds(self):
		return self._east_bounds

	@property
	def polygons(self):
		return self._polygons

	@property
	def center_points(self):
		return self._center_points

	@property
	def heights(self):
		return self._heights

class PathParams:

	def __init__(self, path_params_file):

		self._goal_geodetic = self._read_goal_geodetic(path_params_file)
		self._discretization = self._read_discretization(path_params_file)
		self._safety = self._read_safety_distance(path_params_file)

	def _read_goal_geodetic(self, path_params_file):
		
		print("Reading goal geodetic from file...")

		return goal_geodetic

	def _read_discretization(self, path_params_file):
		
		print("Reading discretization type from file...")

		return discretization

	def _read_safety_distance(self, path_params_file):

		print("Reading safety distance from file...")

		return safety

	@property
	def goal_geodetic(self):
		return self._goal_geodetic

	@property
	def discretization(self):
		return self._discretization

	@property
	def safety(self):
		return self._safety

class Planner:

	def __init__(self, path_params, obstacles):
		
		self._coarse_start_geodetic = None
		self._coarse_start_local = None
		self._coarse_start_grid = None

		self._coarse_goal_geodetic = None
		self._coarse_goal_local = None
		self._coarse_goal_grid = None

		self._local_start_geodetic = None
		self._local_start_local = None
		self._local_start_grid = None

		self._local_goal_geodetic = None
		self._local_goal_local = None
		self._local_goal_grid = None

		if path_params.discretization == Discretization.GRID:
			self._search_space = self._build_2d_grid(obstacles, path_params)
		elif path_params.discretization == Discretization.VORONOI:
			self._search_space = self._build_voronoi_graph(obstacles)
		elif path_params.discretization == Discretization.PRM:
			self._search_space = self._build_prm_graph(obstacles)
		elif path_params.discretization == Discretization.RRT:
			self._search_space = self._build_rrt_graph(obstacles)
		elif path_params.discretization == Discretization.POTENTIAL:
			self._search_space = self._build_potential_field(obstacles)


	def set_coarse_start(self, coarse_start_geodetic):

		print(f"Setting start geodetic to {start_geodetic}")
		
		self.coarse_start_geodetic = start_geodetic
		self.start_local = self.global_to_local()
		self.start_grid = self.local_to_grid()

	def set_goal(self, coarse_goal_geodetic):

		print(f"Setting goal geodetic to {goal_geodetic}")

		self.goal_geodetic = goal_geodetic
		self.goal_local = self.global_to_local(goal_geodetic, home_geodetic)
		self.goal_grid = self.local_to_grid(self.goal_local)


	def global_to_local(geodetic_position, home_geodetic):

		return local_position

	def local_to_grid(self, local_position):

		print("Converting a local position to a grid position...")

		return grid_position


	def build_2d_grid(self, obstacles, altitude):
		
		print("Building a 2d grid search space from obstacle data...")

		return grid_2d
		

	def build_voronoi_graph(self, obstacles):
		
		print("Building a Voronoi graph search space from obstacle data...")
		
		# Build a Voronoi diagram
		
		# Connect valid ridges of diagram to make a graph

		return voronoi_graph
		

	def build_prm_graph(self, obstacles):
		
		print("Building a probabilistic roadmap (PRM) graph search space from obstacle data...")

		return prm_graph
		

	def build_rrt_graph(self, obstacles):
		
		print("Building a rapidly-exploring random tree (RRT) search space from obstacle data...")

		return rrt_graph

	def build_potential_field(self, obstacles):

		print("Building a potential field search space from obstacle data...")

		return potential_field

	def visualize(self):
		
		print("Visualizing the path...")

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

	@property
	def coarse_start_geodetic(self):
		return self._coarse_start_geodetic

	@property
	def coarse_goal_geodetic(self):
		return self._coarse_goal_geodetic

	@property
	def local_start_local(self):
		return self._local_start_local

	@property
	def local_goal_local(self):
		return self._local_goal_local

	@coarse_start_geodetic.setter
	def coarse_start_geodetic(self, coarse_start_geodetic):
		self._coarse_start_geodetic = coarse_start_geodetic

	@coarse_goal_geodetic.setter
	def coarse_goal_geodetic(self, coarse_goal_geodetic):
		self._coarse_goal_geodetic = coarse_goal_geodetic 
	
	@local_start_geodetic.setter
	def local_start_local(self, start):
		self._local_start_local = local_start

	@local_goal_local.setter
	def local_goal_local(self, local_goal):
		self._local_goal_local = local_goal




if __name__ == "__main__":

	# File names
	obstacle_file = "colliders.csv"
	path_params_file = "path_params.json"
	
	# Read obstacle data
	obstacles = Obstacles(obstacle_file)

	# Read path parameters
	path_params = PathParams(path_params_file)

	# Construct a search space
	search_space = SearchSpace(obstacles, path_params)


	# .. #
	# In the plan_path method of MotionPlanning, update search_space's start and goal. 
	# search_space.set_start(current_geodetic->local)
	# search_space.set_goal(next_waypoint)

	# Create a waypoint_finder object
	# waypoint_finder = WaypointFinder(search_space)
	# waypoints = waypoint_finder.search()

	

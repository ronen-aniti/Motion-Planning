from enum import Enum, auto
import time
import numpy as np
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from typing import Tuple, List, Dict
from planning_utils import (read_global_home, build_map_and_take_measurements, 
							read_destinations, global_to_local, collides, 
							calculate_nearest_free_cell_in_2d, euclidean_distance, 
							a_star, remove_collinear, path_to_waypoints)
import pdb

class States(Enum):
	MANUAL = auto()
	ARMING = auto()
	TAKEOFF = auto()
	WAYPOINT = auto()
	LANDING = auto()
	DISARMING = auto()
	PLANNING = auto()

class MotionPlanning(Drone):
	def __init__(self, connection):
		super().__init__(connection)
		self.target_position = np.array([0.0, 0.0, 0.0, 0.0]) # North, East, Altitude, Heading
		self.waypoints = []
		self.in_mission = True
		self.check_state = {}
		self.elevation_map, self.ned_boundaries, self.map_size = build_map_and_take_measurements('colliders.csv')
		self.destinations = read_destinations('destinations.json')
		self.destination = {}

		# initial state
		self.flight_state = States.MANUAL

		# register all callbacks here
		self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
		self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
		self.register_callback(MsgID.STATE, self.state_callback)

	def local_position_callback(self):
		if self.flight_state == States.TAKEOFF:
			if -1.0 * self.local_position[2] > 0.95 * self.destination['alt']:
				self.waypoint_transition()
			elif self.flight_state == States.WAYPOINT:
				drone_speed = np.linalg.norm(self.local_velocity)
				deadband_radius = 0.25 + drone_speed
				if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) <= deadband_radius:
					if len(self.waypoints) > 0:
						self.waypoint_transition()
					else:
						if np.linalg.norm(self.local_velocity) < 1.0:
							self.landing_transition()

	def velocity_callback(self):
		if self.flight_state == States.LANDING:
			if self.global_position[2] - self.global_home[2] < 0.1:
				if abs(self.local_position[2]) < 0.01:
					self.disarming_transition()

	def state_callback(self):
		if self.in_mission:
			if self.flight_state == States.MANUAL:
				self.arming_transition()
			elif self.flight_state == States.ARMING:
				if self.armed:
					self.destination = self.destinations.pop(0)
					self.takeoff_transition()
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
		global_home = read_global_home('colliders.csv')
		self.set_home_position(*global_home)
		print("Arming transition")
		self.take_control()
		self.arm()

	def takeoff_transition(self):
		self.flight_state = States.TAKEOFF
		print("Takeoff transition")
		self.takeoff(self.destination['alt'])

	def waypoint_transition(self):
		self.flight_state = States.WAYPOINT 
		print("Waypoint transition")
		if len(self.waypoints) > 0:
			self.target_position = self.waypoints.pop(0)
		else:
			self.plan_path(self.destination)
		print(f"Target position {self.target_position}")
		self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

	def landing_transition(self):
		self.flight_state = States.LANDING
		print("Landing transition")
		self.land()

	def disarming_transition(self):
		self.flight_state = States.DISARMING
		print("Disarm transition")
		self.disarm()
		self.release_control()

	def plan_path(self, destination: Dict[str, float]) -> None:
		self.flight_state = States.PLANNING
		PREDICTION_HORIZON = 40 # meters
		SAFETY_DISTANCE = 5 # meters 
		
		# Calculate the drone's current position in the NED frame
		current_global_pos = (self._longitude, self._latitude, self._altitude)
		current_local_pos = global_to_local(current_global_pos, self.global_home)

		# Calculate the drone's position in the grid frame
		current_northing_index = int(current_local_pos[0] - self.ned_boundaries[0])
		current_easting_index = int(current_local_pos[1] - self.ned_boundaries[2])
		current_grid_index = (current_northing_index, current_easting_index)

		# Format the destination geodetic coordinates into a tuple, and calculate the goal location in NED.
		goal_global_pos = (destination['lon'], destination['lat'], destination['alt'])
		goal_local_pos = global_to_local(goal_global_pos, self.global_home)
		
		# Calculate an intermediate goal
		direction = (goal_local_pos - current_local_pos) / np.linalg.norm(goal_local_pos - current_local_pos)
		intermediate_goal_local_pos = current_local_pos + direction * PREDICTION_HORIZON

		# Check if intermediate goal is occupied, and if so, reset intermediate goal to be the closest free cell
		intermediate_goal_northing_index = int(intermediate_goal_local_pos[0] - self.ned_boundaries[0])
		intermediate_goal_easting_index = int(intermediate_goal_local_pos[1] - self.ned_boundaries[2])
		intermediate_goal_grid_index = (intermediate_goal_northing_index, intermediate_goal_easting_index)
		intermediate_goal_altitude = -1.0 * intermediate_goal_local_pos[2]

		if collides(self.elevation_map, intermediate_goal_northing_index, intermediate_goal_easting_index, intermediate_goal_altitude):
			intermediate_goal_northing_index, intermediate_goal_easting_index, intermediate_goal_altitude = calculate_nearest_free_cell_in_2d(self.elevation_map, northing_index, easting_index, intermediate_goal_altitude)
			intermediate_goal_grid_index = (intermediate_goal_northing_index, intermediate_goal_easting_index, intermediate_goal_altitude)

		# Run A* to find a grid-frame path from current location to intermediate goal
		path, cost = a_star(self.elevation_map, current_grid_index, intermediate_goal_grid_index, intermediate_goal_altitude, euclidean_distance)
		print(path)

		# Remove the collinear elements from the grid-frame path
		path = remove_collinear(path)
		print(path)
		# Convert the calculated path from grid-frame coordinates to NED waypoints with headings
		waypoints = path_to_waypoints(path, self.ned_boundaries, intermediate_goal_altitude)
		print(waypoints)
		# Set self.waypoints
		self.waypoints = waypoints 

		# Send the waypoints to the simulator
		# self.send_waypoints()



	def send_waypoints(self):
		print("Sending waypoint to simulator...")
		data = msgpack.dumps(self.waypoints)
		self.connection._master.write(data)

	def start(self):
		self.start_log("Logs", "NavLog.txt")
		print("Starting connection")
		super().start()
		self.stop_log()

if __name__ == "__main__":
	conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
	drone = MotionPlanning(conn)
	time.sleep(2)
	drone.start()

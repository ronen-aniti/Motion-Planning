from enum import Enum, auto
import csv
import time
import numpy as np
import msgpack
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from typing import Tuple, List, Dict
from scipy.spatial import distance
from planning_utils import (read_global_home, build_map_and_take_measurements, 
							read_destinations, global_to_local,
							calculate_nearest_free_cell_in_2d,  
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

		# Logging
		self.start_time = time.time()
		self.waypoints_log = []
		self.goal_grid_index = None
		self.current_grid_index = []

		self.target_position = np.array([0.0, 0.0, 0.0, 0.0]) # North, East, Altitude, Heading
		self.waypoints = []
		self.in_mission = True
		self.check_state = {}
		self.elevation_map, self.ned_boundaries, self.map_size = build_map_and_take_measurements('colliders.csv')
		self.destinations = read_destinations('destinations.json')
		self.destination = {}
		self.destination_local_position = None
		
		# initial state
		self.flight_state = States.MANUAL

		# register all callbacks here
		self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
		self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
		self.register_callback(MsgID.STATE, self.state_callback)

	def local_position_callback(self):
		if self.flight_state == States.TAKEOFF:
			if 1.05 * self.destination['alt'] > -1.0 * self.local_position[2] > 0.95 * self.destination['alt']:
				self.waypoint_transition()
		elif self.flight_state == States.WAYPOINT:
			drone_speed = np.linalg.norm(self.local_velocity)
			deadband_radius = 5 #4 + 4 * drone_speed
			if distance.euclidean(self.target_position[0:2], self.local_position[0:2]) < deadband_radius:
				if len(self.waypoints) > 0:
					self.waypoint_transition()
				else:
					if distance.euclidean(self.local_position[0:2], self.destination_local_position[0:2]) <= 1.0:
						self.landing_transition()
					else:
						self.plan_path(self.destination)

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
					self.plan_path(self.destination)
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
		self.takeoff(self.target_position[2])

	def waypoint_transition(self):
		self.flight_state = States.WAYPOINT 
		print("Waypoint transition")
		if len(self.waypoints) > 0:
			self.target_position = self.waypoints.pop(0)
			print(f"Target position {self.target_position}")
			self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])
		else:
			self.plan_path(self.destination)	

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

		PREDICTION_HORIZON = 20 # meters
		
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
		self.destination_local_position = goal_local_pos

		# Calculate an intermediate goal
		direction = (goal_local_pos - current_local_pos) / np.linalg.norm(goal_local_pos - current_local_pos)
		intermediate_goal_local_pos = current_local_pos + direction * PREDICTION_HORIZON

		# Set the intermediate goal to the destination local position if the drone's nearby
		if distance.euclidean(current_local_pos, goal_local_pos) <= PREDICTION_HORIZON:
			interermediate_goal_local_pos = goal_local_pos
			

		# Check if intermediate goal is occupied, and if so, reset intermediate goal to be the closest free cell
		intermediate_goal_northing_index = int(intermediate_goal_local_pos[0] - self.ned_boundaries[0])
		intermediate_goal_easting_index = int(intermediate_goal_local_pos[1] - self.ned_boundaries[2])
		intermediate_goal_grid_index = (intermediate_goal_northing_index, intermediate_goal_easting_index)

		# Set intermediate goal altitude to be the same as the destination altitude
		intermediate_goal_altitude = destination['alt'] 

		# Check if the intermediate goal position is occupied, and, if so, set it to the nearest free position
		if self.elevation_map[intermediate_goal_northing_index][intermediate_goal_easting_index] > intermediate_goal_altitude:
			intermediate_goal_northing_index, intermediate_goal_easting_index = calculate_nearest_free_cell_in_2d(self.elevation_map, intermediate_goal_northing_index, intermediate_goal_easting_index, intermediate_goal_altitude)
			intermediate_goal_grid_index = (intermediate_goal_northing_index, intermediate_goal_easting_index)

		# If the drone's calculating it's position to be in an occupied location, assume that the drone's actually in the nearest free location
		if self.elevation_map[current_northing_index][current_easting_index] > destination['alt']: #modify to current_altitude
			print('OCCUPIED')
			current_grid_index = calculate_nearest_free_cell_in_2d(self.elevation_map, current_northing_index, current_easting_index, destination['alt'])
		
		# Update takeoff altitude
		self.target_position[2] = intermediate_goal_altitude

		# Run A* to find a grid-frame path from current location to intermediate goal
		path, cost = a_star(self.elevation_map, current_grid_index, intermediate_goal_grid_index, destination['alt'], distance.euclidean)
		
		# Remove the collinear elements from the grid-frame path
		path = remove_collinear(path)

		# Convert the calculated path from grid-frame coordinates to NED waypoints with headings
		waypoints = path_to_waypoints(path, self.ned_boundaries, intermediate_goal_altitude)

		# Set self.waypoints
		self.waypoints = waypoints

		# Logging
		print(self.waypoints)
		if path:
			for northing_index, easting_index in path:
				self.waypoints_log.append([easting_index, northing_index])
			self.goal_grid_index = [int(goal_local_pos[1] - self.ned_boundaries[2]), int(goal_local_pos[0] - self.ned_boundaries[0])]
			np_goal_grid_index = np.array(self.goal_grid_index)
			np.save('goal_grid_index.npy', np_goal_grid_index)
			np_waypoints_log = np.array(self.waypoints_log)
			np.save('waypoints_log.npy', np_waypoints_log)
			self.current_grid_index.append([current_easting_index, current_northing_index])
			np_current_grid_index = np.array(self.current_grid_index)
			np.save('current_grid_index.npy', np_current_grid_index)
			binary_occupancy_grid = (self.elevation_map <= destination['alt'])
			np.save('binary_occupancy_grid.npy', binary_occupancy_grid)
		else:
			print(f"CURRENT GRID INDEX, FAILING: {current_grid_index}")
		# Send the waypoints to the simulator
		#self.send_waypoints()




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

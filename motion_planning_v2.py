from enum import Enum, auto
import numpy as np
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from typing import Tuple
from planning_utils import read_global_home, build_map, read_destinations

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
		self.global_home = read_global_home('colliders.csv')
		self.elevation_map = build_elevation_map('colliders.csv')
		self.destinations = read_destinations('destinations.json')

		# initial state
		self.flight_state = States.MANUAL

		# register all callbacks here
		self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
		self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
		self.register_callback(MsgID.State, self.state_callback)

	def local_position_callback(self):
		if self.flight_state == States.TAKEOFF:
			if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
				self.waypoint_transition()
			elif flight_state == States.WAYPOINT:
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
					self.set_home_position(self.global_home[0], self.global_home[1], self.global_home[2])
					self.current_destination = self.destinations.pop(0)
					self.plan_path(self.current_destination)
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
		else:
			self.plan_path(self.current_destination)
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

	def plan_path(self, destination: Tuple[float, float, float]) -> None:
		self.flight_state = States.PLANNING
		PREDICTION_HORIZON = 40

		# Calculate current position in NED frame
		current_geodetic = (self._longitude, self._latitude, self._altitude)
		current_local = global_to_local(current_geodetic, self.global_home)

		# Calculate the destination location in NED
		goal_local = global_to_local(destination, self.global_home)
		
		# Calculate an intermediate goal
		direction = (goal_local - current_local) / np.linalg.norm(goal_local - current_local)
		intermediate_goal = current_local + direction * PREDICTION_HORIZON

		# Check if intermediate goal is occupied, and if so, reset intermediate goal to be the closest free cell

		# Run A* to find a path from start to intermediate goal

		# Convert path to waypoints and set self.waypoints



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

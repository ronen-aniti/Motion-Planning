import pdb
import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import csv

from grid_map import GridMap
from medial_axis_map import MedialAxisGridMap
from reference_frame import global_to_local, local_to_global
import random 

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID

from typing import Tuple, List
from flight_setup import configure_flight_settings, FlightSettings, PlanningAlgorithms
from mapping_utils import read_global_home
from battery_utils import battery_consumption_rate

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()

class MotionPlanning(Drone):

    def __init__(self, connection: MavlinkConnection, flight_settings: FlightSettings):
        super().__init__(connection)


        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        
        # flight settings
        self.planning_algorithm = flight_settings.planning_algorithm
        self.incident_locations = flight_settings.incident_locations
        self.battery_charge = flight_settings.battery_charge

        self.previous_position = None
        self.meters_traveled = 0.0
        self.arm_timestamp = None



        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self) -> None:

        self.update_battery_charge()
        
        # Waypoint transition logic
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            drone_speed = np.linalg.norm(self.local_velocity)
            if self.planning_mode == PlanningAlgorithms.GRID2D:
                deadband_radius = 0.25 + drone_speed
            elif self.planning_mode == PlanningAlgorithms.MEDAXIS:
                deadband_radius = 4.0 + 4.0 * drone_speed
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < deadband_radius:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

                        
    def velocity_callback(self) -> None:
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self) -> None:
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

    def arming_transition(self) -> None:
        self.flight_state = States.ARMING
        print("arming transition")
        self.take_control()
        self.arm()
        self.arm_timestamp = time.time()
        # establish global home
        global_home = read_global_home('colliders.csv')
        self.set_home_position(*global_home)
        print(f"Global home is being set to {global_home}")

    def takeoff_transition(self) -> None:
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])
        print(f"taking off to {self.target_position[2]} m. altitude")

    def waypoint_transition(self) -> None:
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)    
        print('target position', self.target_position) 
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self) -> None:
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self) -> None:
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()
        flight_time = time.time() - self.arm_timestamp
        print(f"Time elapsed while armed: {flight_time:.2f} seconds")
        print(f"Battery state of charge: {self.battery_charge:.2f}%")
        print(f"Total distance traveled: {self.meters_traveled:.2f} meters")

    def manual_transition(self) -> None:
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self) -> None:
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def update_battery_charge(self) -> None:
        # Battery state of charge is a function of distance traveled, velocity, and time armed.
        if self.previous_position is not None and self.arm_timestamp is not None:
            flight_time = time.time() - self.arm_timestamp
            delta_position = self.local_position - self.previous_position
            delta_distance = np.linalg.norm(delta_position[:3])
            consumption_rate = battery_consumption_rate(self.local_velocity)
            time_factor = 100 * 0.025 / 1800
            self.meters_traveled += delta_distance 
            self.battery_charge -= consumption_rate * delta_distance/1000 + time_factor * flight_time

        self.previous_position = self.local_position.copy()

    def plan_path(self) -> None: 

        self.flight_state = States.PLANNING
        current_geodetic = (self._longitude, self._latitude, self._altitude)
        current_local = global_to_local(current_geodetic, self.global_home)
        if len(incident_locations) > 0:
            goal_geodetic = incident_locations.pop(0)
        else:
            print("There are no more incidents to track.")
            return

        destination_local = global_to_local(goal_geodetic, self.global_home)

        SAFETY_DISTANCE = 5.0

        self.target_position[2] = destination_local[2] 

        
        # Select the planning mode based on the flight_subinterval
        planning_algorithms = {
            PlanningAlgorithms.GRID2D: lambda: GridMap('colliders.csv', '2d Grid at 10 m Altitude', SAFETY_DISTANCE, self.global_home, current_local, destination_local).search_grid(),
            PlanningAlgorithms.MEDAXIS: lambda: MedialAxisGridMap('colliders.csv', '2d Medial Axis Grid at 10 m Altitude', SAFETY_DISTANCE, self.global_home, current_local, destination_local).search_grid()
        }
        if self.planning_algorithm not in planning_algorithms:
            raise ValueError(f"Invalid planning mode: {self.planning_algorithm}")
        plan_fn = planning_algorithms[self.planning_algorithm]
        
        print(f"Current global position: {current_geodetic}")
        print(f"Commanded destination: {self.goal_geodetic}")
        print(f"Path planning algorithm selected: {self.planning_algorithm}")
        print("Attempting to compute flight path...")

        

        self.waypoints = plan_fn()

        #print(self.waypoints)

        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        # This line of code seems to be causing problems, so I'm going to comment it out.
        # self.send_waypoints()


    def battery_check(self) -> None:
        if self.battery_charge <= 10:
            print(f"Low battery warning: {self.battery_charge:.2f}% remaining")
            # Implement return to home or emergency landing logic here.


    def start(self) -> None:
        self.start_log("Logs", "NavLog.txt")
        print("starting connection") 
        super().start()
        self.stop_log()


if __name__ == "__main__":

    flight_settings = configure_flight_settings()
    print(flight_settings)

    # Establish a connection with the drone simulator
    conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
    drone = MotionPlanning(conn, flight_settings)
    time.sleep(2)
    drone.start()

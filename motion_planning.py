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

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()

class PlanningModes(Enum):
    GRID2D = 1
    MEDAXIS = 2
    POTFIELD = 3
    VOXEL = 4
    VORONOI = 5
    PRM = 6
    RRT = 7


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL
        self.planning_mode = None
        self.flight_subinterval = 0

        # destination sequence
        self.destination_sequence = [[-122.396375, 37.793913, 10],
                                     [-122.397168, 37.793840,  10]]

        

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            drone_speed = np.linalg.norm(self.local_velocity)
            if self.planning_mode == PlanningModes.GRID2D:
                if drone_speed < 1:
                    deadband_radius = 0.25
                else:
                    deadband_radius = drone_speed
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < deadband_radius:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        #self.landing_transition()
                        self.planning_mode_transition()

    def planning_mode_transition(self):
        print("planning mode transition")
        self.plan_path()

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
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.take_control()
        self.arm()
        # establish global home
        global_home = self.read_global_home('colliders.csv')
        self.set_home_position(*global_home)
        print(f"global home is being set to {global_home}")

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])
        print(f"taking off to {self.target_position[2]} m. altitude")

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self): 
        self.flight_state = States.PLANNING
        subgoal = self.destination_sequence.pop(0)
        subgoal_local = global_to_local(subgoal, self.global_home)
        self.target_position[2] = subgoal_local[2]
        SAFETY_DISTANCE = 5.0
        current_global_position = [self._longitude, self._latitude, self._altitude]
        current_local_position = global_to_local(current_global_position, self.global_home)
        self.flight_subinterval += 1
        self.planning_mode = PlanningModes(self.flight_subinterval)
        print("planning flight path...")
        print(f"Current global position: {current_global_position}")
        print(f"Commanded destination: {subgoal}")
        print(f"Planning algorithm: {self.planning_mode}")
        print("computing flight path...")
        if self.planning_mode == PlanningModes.GRID2D:
            grid_2d = GridMap('colliders.csv', '2d Grid at 10 m Altitude', SAFETY_DISTANCE, self.global_home, current_local_position, subgoal_local)
            self.waypoints = grid_2d.search_grid()
        elif self.planning_mode == PlanningModes.MEDAXIS:
            medaxis_grid = MedialAxisGridMap('colliders.csv', '2d Medial Axis Grid at 10 m Altitude', SAFETY_DISTANCE, self.global_home, current_local_position, subgoal_local)
            self.waypoints = medaxis_grid.search_grid()
        print("path determined... the waypoint sequence is this:")
        print(self.waypoints)





        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        #self.send_waypoints()
    
    def read_global_home(self, filename: str):
        """
        Read in global home from CSV 2.5d map
        """
        with open(filename) as f:
            reader = csv.reader(f)
            line_1 = next(reader)
        lat0 = float(line_1[0].split('lat0 ')[1])
        lon0 = float(line_1[1].split(' lon0 ')[1])

        return (lon0, lat0, 0.0)

    def start(self):
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        super().start()
        self.stop_log()


if __name__ == "__main__":
    conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
    drone = MotionPlanning(conn)
    time.sleep(2)
    drone.start()

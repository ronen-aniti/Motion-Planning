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

    def __str__(self):
        if self == PlanningModes.GRID2D:
            return "2D grid and A* search"
        elif self == PlanningModes.MEDAXIS:
            return "Medial Axis grid with A* search"
        elif self == PlanningModes.POTFIELD:
            return "Potential field"
        elif self == PlanningModes.VOXEL:
            return "Voxel map with A* search"
        elif self == PlanningModes.VORONOI: 
            return "Voronoi graph with A*"
        elif self == PlanningModes.PRM:
            return "Probabilistic Road Map (PRM)"
        elif self == PlanningModes.RRT:
            return "Rapidly-Exploring Random Tree (RRT)"


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # planning mode
        #self.planning_mode = PlanningModes.GRID2D
        self.planning_mode = PlanningModes.MEDAXIS

        # destination geodetic
        self.destination_geodetic = (-122.396375, 37.793913, 10) # California St. and Main St.

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
                deadband_radius = 0.25 + drone_speed
            elif self.planning_mode == PlanningModes.MEDAXIS:
                deadband_radius = 1 + 2 * drone_speed
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < deadband_radius:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
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
        try:
            global_home = self.read_global_home('colliders.csv')
            self.set_home_position(*global_home)
            print(f"Global home is being set to {global_home}")
        except Exception as e:
            print(f"Error during arming transition: {e}")
            self.manual_transition

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
        current_geodetic = (self._longitude, self._latitude, self._altitude)
        current_local = global_to_local(current_geodetic, self.global_home)
        destination_local = global_to_local(self.destination_geodetic, self.global_home)

        SAFETY_DISTANCE = 5.0

        self.target_position[2] = destination_local[2] #Not sure if I need this.
        
        # Select the planning mode based on the flight_subinterval
        planning_modes = {
            PlanningModes.GRID2D: lambda: GridMap('colliders.csv', '2d Grid at 10 m Altitude', SAFETY_DISTANCE, self.global_home, current_local, destination_local).search_grid(),
            PlanningModes.MEDAXIS: lambda: MedialAxisGridMap('colliders.csv', '2d Medial Axis Grid at 10 m Altitude', SAFETY_DISTANCE, self.global_home, current_local, destination_local).search_grid()
        }
        if self.planning_mode not in planning_modes:
            raise ValueError(f"Invalid planning mode: {self.planning_mode}")
        plan_fn = planning_modes[self.planning_mode]
        
        print(f"Current global position: {current_geodetic}")
        print(f"Commanded destination: {self.destination_geodetic}")
        print(f"Path planning algorithm selected: {self.planning_mode}")
        print("Attempting to compute flight path...")

        # Compute the waypoints using the selected planning mode
        self.waypoints = plan_fn()

        print("Path determined... The waypoint sequence is this:")
        print(self.waypoints)

        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        # This line of code seems to be causing problems, so I'm going to comment it out.
        # self.send_waypoints()
    
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

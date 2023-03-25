from typing import Tuple, List
from enum import Enum, auto

class FlightSettings:
    def __init__(self, incident_locations: List[Tuple[float, float, float]], planning_algorithm: PlanningAlgorithms, 
                 multiple_incidents: bool, battery_charge: float):
        self.incident_locations = incident_locations
        self.planning_algorithm = planning_algorithm
        self.battery_charge = battery_charge

    def __str__(self):
        return f"Incident locations: {self.incident_locations};\nPlanning algorithm: {self.planning_algorithm};\n\
                Battery charge: {self.battery_charge:.2f}%"

def configure_flight_settings() -> FlightSettings:

    # Present user with a welcome message.
    print("Welcome.")
    run_default_scenario = bool(int(input("Select whether or not you'd like to run the default scenario. \
        1 = YES, 0 = NO \n")))
    if run_default_scenario:
        incident_locations = [
            (-122.396375, 37.793913, 10),
            (-122.397956, 37.794955, 8)]
    else:
        print("Ok, set another destination.")
        incident_lon = float(input("Input desired longitude "))
        incident_lat = float(input("Input desired latitude "))
        incident_alt = float(input("Input desired altitude "))
        incident_locations = [(goal_lon, goal_lat, goal_alt)]

    print("Select a path planning algorithm.")
    print(f"1= {PlanningAlgorithms.GRID2D}")
    print(f"2= {PlanningAlgorithms.MEDAXIS}")
    planning_algorithm = PlanningAlgorithms(int(input("")))

    battery_charge = 100 

    flight_settings = FlightSettings(incident_locations, planning_algorithm, battery_charge)
    
    return flight_settings


class PlanningAlgorithms(Enum):
    GRID2D = 1
    MEDAXIS = 2
    POTFIELD = 3
    VOXEL = 4
    VORONOI = 5
    PRM = 6
    RRT = 7

    def __str__(self):
        if self == PlanningAlgorithms.GRID2D:
            return "2D grid and A* search"
        elif self == PlanningAlgorithms.MEDAXIS:
            return "Medial Axis grid with A* search"
        elif self == PlanningAlgorithms.POTFIELD:
            return "Potential field"
        elif self == PlanningAlgorithms.VOXEL:
            return "Voxel map with A* search"
        elif self == PlanningAlgorithms.VORONOI: 
            return "Voronoi graph with A*"
        elif self == PlanningAlgorithms.PRM:
            return "Probabilistic Road Map (PRM)"
        elif self == PlanningAlgorithms.RRT:
            return "Rapidly-Exploring Random Tree (RRT)"
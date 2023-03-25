from typing import Tuple, List
from planning_modes import PlanningModes

class FlightSettings:
    def __init__(self, goal_geodetic: Tuple[float, float, float], planning_mode: PlanningModes, 
                 multiple_incidents: bool, battery_charge: float):
        self.goal_geodetic = goal_geodetic
        self.planning_mode = planning_mode
        self.multiple_incidents = multiple_incidents
        self.battery_charge = battery_charge

    def __str__(self):
        return "\nFlight settings\n" \
               f"Goal geodetic: {self.goal_geodetic}\n" \
               f"Planning mode: {self.planning_mode}\n" \
               f"Respond to multiple incidents: {self.multiple_incidents}\n" \
               f"Battery charge: {self.battery_charge}%\n"
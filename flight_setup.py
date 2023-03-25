from planning_modes import PlanningModes

from typing import Tuple, List

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

def configure_flight_settings() -> FlightSettings:

    # Present user with a welcome message.
    print("Welcome. Help the quadcopter plan it's path by responding to this simple sequence of path planning prompts.")

    # Prompt user to select a destination.
    print("Do you intend to have the quadcopter to fly to its default destination, Main St. and California St.?")
    use_default_destination = bool(int(input("(1 = YES, 0 = NO) ")))
    if use_default_destination:
        goal_geodetic = (-122.396375, 37.793913, 10) # California St. and Main St.
    else: 
        print("Ok, set another destination.")
        goal_lon = float(input("Input desired longitude "))
        goal_lat = float(input("Input desired latitude "))
        goal_alt = float(input("Input desired altitude "))
        goal_geodetic = (goal_lon, goal_lat, goal_alt)

    # Prompt user to select a path planning algorithm
    print("Choose a path planning algorithm from this list:")
    print(f"1= {PlanningModes.GRID2D}")
    print(f"2= {PlanningModes.MEDAXIS}")
    planning_mode = PlanningModes(int(input("")))

    # Prompt user to select whether or not to respond to multiple incidents
    print("Select whether or not to respond to multiple events")
    multiple_incidents = bool(int(input("1 = YES, 0 = NO ")))
    
    # Prompt user to select the state of charge of the drone. Logic in code must decide whether to emergency land or to fly back to home base to charge, then complete mission.
    print("Select the state of charge of the drone's battery (as a percentage)")
    print("100 indicates full charge. Full charge is equivalent to 2,500 meter range")
    print("0 indicates no charge")
    battery_charge = float(input(""))

    flight_settings = FlightSettings(goal_geodetic, planning_mode, multiple_incidents, battery_charge)
    
    return flight_settings
\
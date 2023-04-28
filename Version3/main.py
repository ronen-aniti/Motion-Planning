from environment import Environment
from geodetic_position import GeodeticPosition
from start_goal_pair import StartGoalPair
from state import State
from obstacle_collection import ObstacleCollection
from obstacle_file_reader import ObstacleFileReader
from state_samplers_v2 import RapidlyExploringRandomTree
import pdb

"""
The conceptual organization of this code is like this.
Environment 
-Obstacles
-Bounds
-Home geodetic position
-Goal geodetic position
-Goal state


State
-Local position
-Heading
-Waypoint command
-Associated environment
-Ground distance to goal
-Ground position
-3d position
-3d position to goal
"""
filename = "colliders.csv"
reader = ObstacleFileReader(filename) 
geodetic_home = reader.extract_geodetic_home()
obstacle_array = reader.extract_obstacles_as_array() 
obstacle_collection = ObstacleCollection(obstacle_array, geodetic_home)
geodetic_goal = GeodeticPosition(-122.396375, 37.793913, 10) #User determines goal lon, lat, alt.
environment = Environment(geodetic_home, obstacle_collection)
current_geodetic_position = GeodeticPosition(-122.39745, 37.79248, 0) #The geodetic position of the drone at the position update
current_local_position = current_geodetic_position.local_relative_to(geodetic_home)
goal_position_in_local_frame = geodetic_goal.local_relative_to(geodetic_home)

#environment.visualize()
current_state = State(environment, goal_position_in_local_frame, current_local_position)
goal_state = State(environment, goal_position_in_local_frame, goal_position_in_local_frame)

rapidly_exploring_random_tree = RapidlyExploringRandomTree(environment, current_state, goal_state)

rapidly_exploring_random_tree.visualize()
minimum_cost_state_path = rapidly_exploring_random_tree.search_for_minimum_cost_path_of_states()
rapidly_exploring_random_tree.visualize(show_solution=True)
#sequence_of_states_from_start_to_goal = rapidly_exploring_random_tree.determine_state_sequence(current_state, goal_state, environment)
"""
# Driver code
filename = "colliders.csv"
reader = ObstacleFileReader(filename) # -> extract_global_home() -> State from GeodeticPosition,  
geodetic_home = reader.extract_geodetic_home() #-> GeodeticPosition 

obstacle_array = reader.extract_obstacles_as_array() #-> np.ndarray
obstacle_collection = ObstacleCollection(obstacle_array, geodetic_home) # .list List[Obstacle]; .tree KDtree
geodetic_goal = GeodeticPosition(-122.3123, 37.1231, 10) # modify the exact position floats
environment = Environment(geodetic_home, geodetic_goal, obstacle_collection)
global_start_goal_pairing = StartGoalPair(geodetic_home, geodetic_goal)
global_sampler = GlobalSampler(global_start_goal_pairing) #Global sampler object with global plan method
global_state_seqeunce = global_sampler.determine_global_state_sequence() # List[State]
waypoints = global_sampler.convert_state_sequence_to_waypoints(global_state_sequence) #List[float]
# Send the state sequence to the drone as waypoint list

# Upon position update...
## Potentially simulate adding an obstacle into the obstacle collection using obstacle_collection.insert_obstacle_into_list(new_obstacle)
longitude, latitude, altitude = -122.1111, 37.222, 9.827 # Get these from drone's GPS
geodetic_position = GeodeticPosition(longitude, latitude, altitude)
target_geodetic_position = global_state_sequence[target_state_index].geodetic_position # Increment target state sequence each time the drone's within deadband radius of target
local_start_goal_pairing = StartGoalPair(geodetic_position, target_geodetic_position)
local_sampler = LocalSampler(local_start_goal_pairing)
next_waypoint = local_sampler.determine_next_state().waypoint
"""

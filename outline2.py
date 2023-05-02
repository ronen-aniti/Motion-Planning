from obstacle_file_reader import ObstacleFileReader
import pdb

# Driver code
filename = "colliders.csv"
reader = ObstacleFileReader(filename) # -> extract_global_home() -> State from GeodeticPosition,  
geodetic_home = reader.extract_geodetic_home() #-> GeodeticPosition 
pdb.set_trace()
"""
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

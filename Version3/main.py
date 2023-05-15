from environment import Environment
from geodetic_position import GeodeticPosition
from graph import Graph
from grid import Grid
from halfsize import HalfSize
from local_position import LocalPosition
import matplotlib.pyplot as plt
from prm import ProbabilisticRoadmap
from start_goal_pair import StartGoalPair
from state import State
from obstacle import Obstacle
from obstacle_collection import ObstacleCollection
from obstacle_file_reader import ObstacleFileReader
from rrt import RapidlyExploringRandomTree
from potential_field import PotentialField
import time
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

# Record a list of geodetic coordinates
# Run the global planner to the first destination
# Update the local planner every so often or every time an obstacle is detected
# Add an obstacle and demonstrate the local planners ability to navigate around it
# When the destination is reached, start repeat this process with the new destination.
# When the battery is low, estimate the meters required to get home. If there's enough range, then go home. If not, then continue to next waypoint until range is very low, then land. 

#def code_that_I_want_to_time():
filename = "colliders.csv"
reader = ObstacleFileReader(filename) 
geodetic_home = reader.extract_geodetic_home()
obstacle_array = reader.extract_obstacles_as_array() 
obstacle_collection = ObstacleCollection(obstacle_array, geodetic_home)

##
# Try adding an obstacle into the collection
print("Adding...")
obs_local_pos = LocalPosition(220, -90, 100)
obs_halfsize = HalfSize(20, 20, 100)
obs = Obstacle(obs_local_pos, obs_halfsize)
obstacle_collection.insert_obstacle_into_collection(obs)
print("Obstacle added")
##

#geodetic_goal = GeodeticPosition(-122.396375, 37.793913, 10) #User determines goal lon, lat, alt.
geodetic_goal = GeodeticPosition(-122.3994, 37.7951, 10)
environment = Environment(geodetic_home, obstacle_collection)
current_geodetic_position = GeodeticPosition(-122.39745, 37.79248, 0) #The geodetic position of the drone at the position update
current_local_position = current_geodetic_position.local_relative_to(geodetic_home)
goal_position_in_local_frame = geodetic_goal.local_relative_to(geodetic_home)

current_state = State(environment, goal_position_in_local_frame, current_local_position)
goal_state = State(environment, goal_position_in_local_frame, goal_position_in_local_frame)


# Generate a rapidly-exploring random tree (RRT)
rapidly_exploring_random_tree = RapidlyExploringRandomTree(environment, current_state, goal_state)
waypoints = rapidly_exploring_random_tree.run() 
rapidly_exploring_random_tree.visualize(plot_entire_state_space=True)

# Generate a potential field (actually, a gradient field) trajectory
potential_field = PotentialField(environment, current_state, goal_state)
waypoints = potential_field.return_waypoints()
potential_field.visualize(plot_with_gradient_field=True, plot_original_trajectory=True, plot_shortened_path=True)

# Generate a probabilistic roadmap (a graph)
# TODO: Write a path shortening algorithm for the PRM planner
prm = ProbabilisticRoadmap(environment, current_state, goal_state)
waypoints = prm.determine_waypoints()
prm.visualize(plot_entire_state_space=True)

# Generate a 2d graph representation
graph = Graph(environment, current_state, goal_state)
waypoints = graph.search()
print(waypoints)
graph.visualize(plot_entire_state_space=True)

# Generate a 2d grid representation
grid = Grid(environment, current_state, goal_state)
grid.search()
grid.visualize()
 
# Plot the trajectory for the potential field planner
#fig, ax = environment.visualize()
#potential_field_trajectory = [state.local_position for state in potential_field.state_sequence]
#ax.plot([p.north for p in potential_field_trajectory], [p.east for p in potential_field_trajectory], label="Potential Field Trajectory")

#ax.legend()
#plt.show()


	#return rapidly_exploring_random_tree.current_iter
"""
n = 100

times = []
iterations = []
for i in range(n):
	start_time = time.time()
	maximum_number_of_iterations = code_that_I_want_to_time()
	end_time = time.time()
	times.append(end_time - start_time)
	iterations.append(maximum_number_of_iterations)
avg_time = sum(times) / n 
print("Average time: ", avg_time)
print("Iterations ", iterations)
"""
#rapidly_exploring_random_tree.visualize()
#minimum_cost_state_path = rapidly_exploring_random_tree.search_for_minimum_cost_path_of_states()
#rapidly_exploring_random_tree.visualize(show_solution=True)
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

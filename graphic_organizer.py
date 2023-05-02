import csv
import numpy as np
import utm
from sklearn.neighbors import KDTree
import random 
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.patches import Polygon
import networkx as nx
import itertools
import pdb




class Obstacle:

	def __init__(self, center_north, center_east, center_height, half_size_north, half_size_east, half_size_height):
		self._base_center_coord = np.array([center_north, center_east])
		self._center_coord_3d = np.array([center_north, center_east, center_height])
		self._center_height = center_height
		self._height = center_height + half_size_height
		self._half_size_north = half_size_north
		self._half_size_east = half_size_east
		self._half_size_height = half_size_height
		self._north_min = center_north - half_size_north
		self._north_max = center_north + half_size_north
		self._east_min = center_east - half_size_east
		self._east_max = center_east + half_size_east
		self._base_coord_1 = np.array([self._north_min, self.east_min])
		self._base_coord_2 = np.array([self._north_min, self.east_max])
		self._base_coord_3 = np.array([self._north_max, self.east_max])
		self._base_coord_4 = np.array([self._north_max, self.east_min])
		self._radius = np.hypot(half_size_north, half_size_east)

	def __str__(self):
		center_north, center_east = (f"{coord:.2f}" for coord in self._base_center_coord)
		height = f"{self._height:0.2f}"
		return (f"Obstacle object with base center coordinate ({center_north}, {center_east}) "
				f"and height {height}")

	@property
	def base_center_coord(self):
		return self._base_center_coord

	@property
	def center_coord_3d(self):
		return self._center_coord_3d

	@property
	def center_height(self):
		return self._center_height

	@property
	def height(self):
		return self._height

	@property 
	def half_size_north(self):
		return self._half_size_north

	@property
	def half_size_east(self):
		return self._half_size_east

	@property
	def half_size_height(self):
		return self._half_size_height

	@property
	def north_min(self):
		return self._north_min

	@property
	def north_max(self):
		return self._north_max

	@property
	def east_min(self):
		return self._east_min

	@property
	def east_max(self):
		return self._east_max

	@property
	def base_coord_1(self):
		return self._base_coord_1

	@property
	def base_coord_2(self):
		return self._base_coord_2

	@property
	def base_coord_3(self):
		return self._base_coord_3

	@property
	def base_coord_4(self):
		return self._base_coord_4

	@property
	def radius(self):
		return self._radius



class Environment:
	def __init__(self, filename):

		self._global_home_lon = self._read_home_lon(filename)
		self._global_home_lat = self._read_home_lat(filename)
		self._global_home_alt = 0.0
		
		# north, east, alt, dnorth, deast, dalt
		self._raw_obstacle_data = self._read_raw_obstacle_data(filename)
		self._number_of_obstacles = self.raw_obstacle_data.shape[0]

		self._north_min = self._extract_north_min(self._raw_obstacle_data)
		self._north_max = self._extract_north_max(self._raw_obstacle_data)
		self._east_min = self._extract_east_min(self._raw_obstacle_data)
		self._east_max = self._extract_east_max(self._raw_obstacle_data)
		self._alt_min = self._extract_alt_min(self._raw_obstacle_data)
		self._alt_max =  self._extract_alt_max(self._raw_obstacle_data)

		self._north_bounds = (self._north_min, self._north_max)
		self._east_bounds = (self._east_min, self._east_max)
		self._alt_bounds = (self._alt_min, self._alt_max)
		
		self._max_radius = self._determine_max_radius(self._raw_obstacle_data)

		self._obstacles = self._build_obstacle_array()
		self._obstacle_kdtree = KDTree([obstacle.base_center_coord for obstacle in self.obstacles])

		self._graph = None 

	def __str__(self):
		lon, lat, alt = (f"{position:.4f}" for position in (self._global_home_lon, self._global_home_lat, self._global_home_alt))
		north_min, north_max = (f"{position:.2f}" for position in self._north_bounds)
		east_min, east_max = (f"{position:.2f}" for position in self._east_bounds)
		alt_min, alt_max = (f"{position:.2f}" for position in self._alt_bounds)
		max_radius = f"{self._max_radius:.2f}"
		number_of_obstacles = f"{self._number_of_obstacles}"
		return (f"Environment object with {number_of_obstacles} obstacles; "
				f"with home geodetic coordinates ({lon}, {lat}, {alt}); "
				f"with obstacle center point north bounds "
				f"({north_min}, {north_max}), east bounds ({east_min}, {east_max}), "
				f"and altitude bounds ({alt_min}, {alt_max}); and with a minimum safe "
				f"flying distance, perpendicular to the height axis passing through "
				f"the center points of the obstacles, of {max_radius} meters")

	def _read_home_lon(self, filename):

		print("Reading home longitude from file...")

		with open(filename) as csvfile:
			reader = csv.reader(csvfile)
			first_line = next(reader) # Read the first row of the file
			global_home_lon = float(first_line[1].split(' lon0 ')[1])

		return global_home_lon 


	def _read_home_lat(self, filename):

		print("Reading home latitude from file...")

		with open(filename, newline='') as csvfile:
			reader = csv.reader(csvfile)
			first_line = next(reader) # Read the first row of the file
			global_home_lat = float(first_line[0].split('lat0 ')[1])

		return global_home_lat

	def _read_raw_obstacle_data(self, filename):
		
		print("Reading obstacle data from file...")

		raw_obstacle_data = np.loadtxt(filename, delimiter=',', skiprows=2)

		return raw_obstacle_data

	def _extract_north_min(self, raw_obstacle_data):

		print("Extracting the minimum obstacle center position...")
		
		north_min = np.min(raw_obstacle_data[:, 0])

		return north_min

	def _extract_east_min(self, raw_obstacle_data):

		print("Extracting the minimum east obstacle center position...")
		
		east_min = np.min(raw_obstacle_data[:, 1])

		return east_min

	def _extract_north_max(self, raw_obstacle_data):
		
		print("Extracting the maximum north obstacle center position...")

		north_max = np.max(raw_obstacle_data[:, 0])

		return north_max

	def _extract_east_max(self, raw_obstacle_data):	

		print("Extracting the maximum east obstacle center position...")

		east_max = np.max(raw_obstacle_data[:, 1])

		return east_max

	def _extract_alt_min(self, raw_obstacle_data):
		
		print("Extracting the minimum altitude obstacle center position...")

		alt_min = np.min(raw_obstacle_data[:, 2])

		return alt_min

	def _extract_alt_max(self, raw_obstacle_data):
		
		print("Extracting the maximum altitude obstacle center position...")

		alt_max = np.max(raw_obstacle_data[:, 2])

		return alt_max

	def _determine_max_radius(self, raw_obstacle_data):
		
		print(f"Determining the minimum safe flying distance around obstacle, "
			  f"perpendicular to the height axis passing through the center point...")

		max_half_size_north = np.max(raw_obstacle_data[:, 3])
		max_half_size_east = np.max(raw_obstacle_data[:, 4])
		max_radius = np.hypot(max_half_size_north, max_half_size_east)

		return max_radius

	def collides(self, sample):
		sample_ground_position = np.array([sample.north, sample.east])
		sample_height = sample.altitude

		indices_within_radius = self._obstacle_kdtree.query_radius(sample_ground_position.reshape(1, -1), r=self.max_radius)

		if len(indices_within_radius[0]) > 0:
			if sample_height < self._obstacles[indices_within_radius[0][0]].height:
				return True

		return False

	def _build_obstacle_array(self):
	
		obstacles = []
		
		for i in range(self.number_of_obstacles):
			center_north, center_east, center_height, half_size_north, half_size_east, half_size_height = self.raw_obstacle_data[i, :]
			obstacles.append(Obstacle(center_north, center_east, center_height, half_size_north, half_size_east, half_size_height))

		return obstacles

	@property
	def graph(self):
		return self._graph
	
	@graph.setter
	def graph(self, graph):
		self._graph = graph

	@property
	def global_home_lon(self):
		return self._global_home_lon

	@property
	def global_home_lat(self):
		return self._global_home_lat

	@property
	def global_home_alt(self):
		return self._global_home_alt

	@property
	def north_min(self):
		return self._north_min

	@property
	def north_max(self):
		return self._north_max

	@property
	def east_min(self):
		return self._east_min

	@property
	def east_max(self):
		return self._east_max

	@property
	def alt_min(self):
		return self._alt_min

	@property
	def alt_max(self):
		return self._alt_max

	@property
	def north_bounds(self):
		return self._north_bounds

	@property
	def east_bounds(self):
		return self._east_bounds

	@property
	def alt_bounds(self):
		return self._alt_bounds

	@property
	def max_radius(self):
		return self._max_radius

	@property
	def raw_obstacle_data(self):
		return self._raw_obstacle_data

	@property
	def number_of_obstacles(self):
		return self._number_of_obstacles

	@property
	def obstacles(self):
		return self._obstacles

	@property
	def obstacles_kdtree(self):
		return self._obstacles_kdtree



def sample_state(environment):
	north = random.uniform(environment.north_min, environment.north_max)
	east = random.uniform(environment.east_min, environment.east_max)
	alt = random.uniform(environment.alt_min, environment.alt_max)
	heading = 0.0

	sample = State(north, east, alt, heading)

	return sample



def plot_environment(environment, start_state, goal_state, more_states):
	fig, ax = plt.subplots()

	# Combine the obstacle and state altitudes for normalization
	all_altitudes = [start_state.altitude] + [state.altitude for state in more_states] + [obstacle.height for obstacle in environment.obstacles]
	min_altitude = min(all_altitudes)
	max_altitude = max(all_altitudes)
	norm = mcolors.Normalize(vmin=min_altitude, vmax=max_altitude)
	cmap = plt.cm.summer

	# Plot obstacles with colors representing their altitudes
	for obstacle in environment.obstacles:
		rectangle_coords = [(obstacle.north_min, obstacle.east_min),
							(obstacle.north_min, obstacle.east_max),
							(obstacle.north_max, obstacle.east_max),
							(obstacle.north_max, obstacle.east_min)]
		rectangle = Polygon(rectangle_coords, closed=True,
							edgecolor='r', facecolor=cmap(norm(obstacle.height)), alpha=0.4)
		ax.add_patch(rectangle)

	# Plot starting state
	ax.scatter(start_state.north, start_state.east, c=cmap(norm(start_state.altitude)), marker='o', label='Start', s=20)
	ax.text(start_state.north, start_state.east, "Start", ha="center", va="bottom")
	# Plot the goal state
	ax.scatter(goal_state.north, goal_state.east, c=cmap(norm(goal_state.altitude)), marker='o', label='Goal', s=20)
	ax.text(goal_state.north, goal_state.east, "Goal", ha="center", va="bottom")

	# Plot additional states
	for state in more_states:
		ax.scatter(state.north, state.east, c=cmap(norm(state.altitude)), marker='o', s=5)

	# Add a color bar to show altitude values
	sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
	sm.set_array([])
	cbar = plt.colorbar(sm, ax=ax)
	cbar.set_label('Altitude')

	for state1, state2 in environment.graph.edges():
		ax.plot([state1.north, state2.north],[state1.east, state2.east], 'k-', alpha=0.5)
		weight = environment.graph[state1][state2]['weight']
		midpoint = ((state1.north + state2.north) / 2, (state1.east + state2.east) / 2)
		ax.text(midpoint[0], midpoint[1], f"{weight:.2f}", fontsize=8)

	ax.set_xlabel('North')
	ax.set_ylabel('East')
	ax.legend()

	plt.show()



def line_3d(P0, V, t):
	return P0 + t * V

def distance_between_states(state1, state2):
	return np.linalg.norm(state2.coord_3d - state1.coord_3d)

def line_collides(env, state1, state2, num_checks=10):
	pass

def build_graph(all_states, environment, k=5):

	G = nx.Graph()

	for state in all_states.all_states:
		G.add_node(state)

	for state_index in range(len(all_states.all_states)):
		state1 = all_states.all_states[state_index]
		distances, indices = all_states.kdtree.query(np.array([state1.north, state1.east]).reshape(1, -1), k=k+1)
		nearest_neighbors = [all_states.all_states[i] for i in indices.flatten() if i != state_index]

		for neighbor in nearest_neighbors:
			P1 = np.array([neighbor.north, neighbor.east, neighbor.altitude])
			P0 = np.array([state1.north, state1.east, state1.altitude])
			V = P1 - P0

			distance = np.linalg.norm(V)
			num_samples = int(np.ceil(distance / 1)) # sample every 1 m
			t_values = np.linspace(0, 1, num_samples)
			hit = False
			for t in t_values:
				north, east, alt = line_3d(P0, V, t)
				s = State(north, east, alt, 0)
				if environment.collides(s):
					hit = True
					break
			if not hit:
				G.add_edge(state1, neighbor, weight=distance)

	return G

class Plan:
	def __init__(self, start_state, goal_state, middle_states):
		self._start_state = start_state
		self._goal_state = goal_state
		self._middle_states = middle_states
		self._all_states = [start_state] + middle_states + [goal_state]
		self._ground_coords = KDTree([np.array([state.north, state.east]) for state in self._all_states])

	@property
	def start_state(self):
		return self._start_state

	@property
	def goal_state(self):
		return self._goal_state

	@property
	def other_states(self):
		return self._goal_state

	@property
	def all_states(self):
		return self._all_states

	@property
	def ground_coords(self):
		return self._ground_coords

class Geodetic:
	def __init__(self, lon, lat, alt, heading=0):
		self._lon = lon
		self._lat = lat
		self._alt = alt
		self._heading = heading

	@property
	def lon(self):
		return self._lon

	@property
	def lat(self):
		return self._lat

	@property
	def alt(self):
		return self._alt

	@property
	def heading(self):
		return self._heading

class State:
	def __init__(self, north, east, altitude, heading, geo_home):
		self._north = north
		self._east = east
		self._altitude= altitude
		self._heading = heading
		self._state = np.array([north, east, altitude, heading])
		self._coord_3d = np.array([north, east, altitude])

		self.geo_home = 


	def __str__(self):
		
		north = f"{self._north:.2f}"
		east = f"{self._east:.2f}"
		altitude = f"{self._altitude:.2f}"
		heading = f"{self._heading:.2f}"
		
		return (f"State object with coordinates {north} m north, "
				f"{east} m east, {altitude} m altitude, and "
				f"{heading} rad. heading")

	@property
	def north(self):
		return self._north

	@property
	def east(self):
		return self._east

	@property
	def altitude(self):
		return self._altitude

	@property
	def heading(self):
		return self._heading

	@property
	def state(self):
		return self._state

	@property
	def coord_3d(self):
		return self._coord_3d
	
class Position:
	def __init__(self, geodetic_position, geodetic_home):
		
		self._geodetic = geodetic_position
		self._local = self._convert_to_local(geodetic_home)

		self._local_state = State(self._local[0], self._local[1], self._local[2], self.local[3])
		self._north = self._local_state.north
		self._east = self._local_state.east
		self._heading = self._local_state.heading



	def _convert_to_local(self, geodetic_home):
		
		home_east, home_north, _, _ = utm.from_latlon(geodetic_home.lat, geodetic_home.lon)
		east, north, _, _ = utm.from_latlon(self._lat, self._lon)
		
		north_delta = north - home_north
		east_delta = east - home_east
		alt_delta = self._alt - global_home.alt
		
		local = np.array([north_delta, east_delta, alt_delta, heading])
		
		return local 

	@property
	def lon(self):
		return self._lon
	@property
	def lat(self):
		return self._lat
	@property
	def alt(self):
		return self._alt
	@property
	def state(self):
		return self._local_state
	@property
	def north(self):
		return self._north
	@property
	def east(self):
		return self._east
	@property
	def heading(self):
		return self._heading
	


def generate_n_states(environment, n):
	i = 0
	states = []
	while i < n:
		sample = sample_state(environment)
		if not environment.collides(sample):
			states.append(sample)
			i += 1
	return states

def generate_n_states_around_position(environment, n, position):
	i = 0
	states = []
	while i < n:
		sample = sample_state

# Establish obstacle file name
filename = "colliders.csv"

# Build environment object
environment = Environment(filename)

# Build array of obstacle objects
obstacles = build_obstacle_array(environment)

# Define a starting state
home_geodetic = Geodetic(-122.397450, 37.792480, 0)
start = Position(-122.3981, 37.7951, 10, home_geodetic)

# Generate some more states
more_states = generate_n_states(environment, 100)

goal_geodetic = (-122.3981, 37.7951, 10)
goal = Position(goal_geodetic, )

# Define a list of all the states
all_states = Samples(start.state, goal.state, more_states)

environment.graph = build_graph(all_states, environment, k=3)


print("DONE")
plot_environment(environment, start.state, goal.state, more_states)
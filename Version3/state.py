from local_position import LocalPosition
import numpy as np
from start_goal_pair import StartGoalPair


class State:
	def __init__(self, environment: "Environment", local_position_of_goal: LocalPosition, local_position: LocalPosition, heading=0, parent_state=None):
		self._local_position = local_position
		self._heading = heading
		self._local_position_of_goal = local_position_of_goal
		self._position_in_3d = np.array([self._local_position.north, self._local_position.east, self._local_position.down])
		self._waypoint = [self._local_position.north, self._local_position.east, -1.0 * self._local_position.down, self._heading] #List[float]
		self._environment = environment
		self._ground_distance_to_goal = self._determine_ground_distance_to_goal()
		self._distance_to_goal = self._determine_distance_to_goal()
		self._ground_position = np.array([self._local_position.north, self._local_position.east])
		self._geodetic_position = self._local_position.convert_to_geodetic_from_local(environment.geodetic_home)
		self._parent_state = parent_state
		if self._parent_state is not None:
			self._distance_to_parent_state = self._position_in_3d - self._parent_state.position_in_3d
		else:
			self._distance_to_parent_state = None

	def __lt__(self, other):
		return id(self) < id(other)


	def _determine_distance_to_nearest_obstacle(self) -> float:
		pass

	def _determine_ground_distance_to_goal(self) -> float:
		ground_distance_between_start_state_and_goal_state = np.linalg.norm([
			self.local_position.north - self._local_position_of_goal.north,
			self.local_position.east - self._local_position_of_goal.east])
		return ground_distance_between_start_state_and_goal_state

	def determine_distance_between_two_states(self, other_state) -> float:
		distance_between_two_states = np.linalg.norm(self._position_in_3d - other.position_in_3d)
		return distance_between_two_states
	
	def _determine_distance_to_goal(self) -> float:
		distance_to_goal = np.linalg.norm(np.array([
			self._local_position.north - self._local_position_of_goal.north,
			self._local_position.east - self._local_position_of_goal.east,
			self._local_position.down - self._local_position_of_goal.down]))
		return distance_to_goal

	def state_collides_with_an_obstacle(self, environment: "Environment") -> bool:
		pass

	@property
	def geodetic_position(self):
		return self._geodetic_position

	@property
	def local_position(self):
		return self._local_position

	@property
	def heading(self):
		return self._heading

	@property
	def waypoint(self):
		return self._waypoint

	@property
	def environment(self):
		return self._environment

	@property
	def ground_distance_to_goal(self):
		return self._ground_distance_to_goal

	@property
	def distance_to_goal(self):
		return self._distance_to_goal

	@property
	def ground_position(self):
		return self._ground_position

	@property
	def position_in_3d(self):
		return self._position_in_3d


	@property
	def parent_state(self):
		return self._parent_state

	@parent_state.setter
	def parent_state(self, new_parent_state):
		self._parent_state = new_parent_state

	@property
	def distance_to_parent_state(self):
		return self._distance_to_parent_state

class PotentialMeshState(State):
	def __init__(self, environment: "Environment", local_position_of_goal: LocalPosition, local_position: LocalPosition, relative_start_state, relative_goal_state, heading=0):
		super().__init__(environment, local_position_of_goal, local_position, relative_start_state, relative_goal_state)
		self._relative_start_state = relative_start_state
		self._relative_goal_state = relative_goal_state
		self._distance_from_relative_start = self._determine_distance_from_relative_start()
		self._distance_from_relative_goal = self._determine_distance_from_relative_goal()
		self._distance_from_nearest_obstacle = self._ground_distance_to_nearest_obstacle()
		self._attractive_potential = self._determine_attractive_potential()
		self._repulsive_potential = self._determine_repulsive_potential()
		self._total_potential = self._attractive_potential + self._repulsive_potential
		self._connections_list = None

	def _determine_distance_from_relative_start(self) -> float:
		return np.linalg.norm(self._position_in_3d - self._relative_start_state.position_in_3d)

	def _determine_distance_from_relative_goal(self) -> float:
		return np.linalg.norm(self._relative_goal_state.position_in_3d - self._position_in_3d)

	def _ground_distance_to_nearest_obstacle(self) -> float:
		distances, indices = self._environment._obstacles.tree.query([self._ground_position], k=5)
		indices = indices[0]
		for index in indices:
			obstacle_to_test = self._environment_obstacles.list[index]
			if obstacle_to_test.height >= self._local_position.down:
				distance_to_nearest_obstacle = np.linalg.norm(obstacle_to_test.ground_position - self._ground_position)
		return 10000000 # Return a large number if none of the nearest neighbor obstacles are actually obstacles at the query state's height.


	def _determine_repulsive_potential(self) -> float:
		k_obstacle = 1
		k_start = 1
		repulsive_potential = (k_obstacle * 1 / (self._distance_from_nearest_obstacle - self._environment.obstacles.safety)**2
							   + k_start * 1 / self._distance_from_relative_start**2)
		return repulsive_potential


	def _determine_attractive_potential(self):
		k_goal = 1
		attractive_potential = -k_goal * 1 / (self._distance_from_relative_goal)**2
		return attractive_potential

	@property
	def total_potential(self):
		return self._total_potential
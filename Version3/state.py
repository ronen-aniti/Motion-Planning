from environment import Environment
from local_position import LocalPosition
import numpy as np
from start_goal_pair import StartGoalPair


class State:
	def __init__(self, environment: Environment, local_position: LocalPosition, heading=0):
		self._local_position = local_position
		self._heading = heading
		self._waypoint = [self._local_position.north, self._local_position.east, -1.0 * self._local_position.down, self._heading] #List[float]
		self._environment = environment
		self._ground_distance_to_goal = self._determine_distance_to_goal()
		self._ground_position = np.array([self._local_position.north, self._local_position.east])
		self._position_in_3d = np.array([self._local_position.north, self._local_position.east, self._local_position.down])

	def _local_to_geodetic(local_position: LocalPosition, environment: Environment):
		pass

	def _geodetic_to_local(self, geodetic_pairing: StartGoalPair) -> np.ndarray:
		pass

	def _determine_distance_to_nearest_obstacle(self) -> float:
		pass

	def _determine_distance_to_goal(self) -> float:
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
	def ground_position(self):
		return self._ground_position

	@property
	def position_in_3d(self):
		return self._position_in_3d
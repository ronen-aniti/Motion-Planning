from bounds import Bounds
from geodetic_position import GeodeticPosition
from local_position import LocalPosition
import numpy as np
from obstacle import Obstacle
from obstacle_collection import ObstacleCollection


class Environment:
	def __init__(self, geodetic_home: GeodeticPosition, geodetic_goal: GeodeticPosition, obstacle_collection: ObstacleCollection):
		self._geodetic_home = geodetic_home
		self._geodetic_goal = geodetic_goal
		self._obstacles = obstacle_collection
		self._north_bounds = self._determine_north_bounds() # Build and return Bounds Object from obstacle_collection
		self._east_bounds = self._determine_east_bounds()
		self._down_bounds = self._determine_down_bounds()

	def _determine_north_bounds(self) -> Bounds:
		north_minimum_environment = np.inf
		for obstacle in self._obstacles.list:
			north_minimum_obstacle = obstacle.local_position.north - obstacle.halfsize.north
			if north_minimum_obstacle < north_minimum_environment:
				north_minimum_environment = north_minimum_obstacle

		north_maximum_environment = -np.inf
		for obstacle in self._obstacles.list:
			north_maximum_obstacle = obstacle.local_position.north + obstacle.halfsize.north
			if north_maximum_obstacle > north_maximum_environment:
				north_maximum_environment = north_maximum_obstacle

		north_bounds = Bounds(north_minimum_environment, north_maximum_environment)

		return north_bounds

	def _determine_east_bounds(self) -> Bounds:
		east_minimum_environment = np.inf
		for obstacle in self._obstacles.list:
			east_minimum_obstacle = obstacle.local_position.east - obstacle.halfsize.east
			if east_minimum_obstacle < east_minimum_environment:
				east_minimum_environment = east_minimum_obstacle

		east_maximum_environment = -np.inf
		for obstacle in self._obstacles.list:
			east_maximum_obstacle = obstacle.local_position.east + obstacle.halfsize.east
			if east_maximum_obstacle > east_maximum_environment:
				east_maximum_environment = east_maximum_obstacle

		east_bounds = Bounds(east_minimum_environment, east_maximum_environment)

		return east_bounds

	def _determine_down_bounds(self) -> Bounds:
		down_minimum_environment = np.inf
		for obstacle in self._obstacles.list:
			down_minimum_obstacle = obstacle.local_position.down - obstacle.halfsize.down
			if down_minimum_obstacle < down_minimum_environment:
				down_minimum_environment = down_minimum_obstacle

		down_maximum_environment = -np.inf
		for obstacle in self._obstacles.list:
			down_maximum_obstacle = obstacle.local_position.down + obstacle.halfsize.down
			if down_maximum_obstacle > down_maximum_environment:
				down_maximum_environment = down_maximum_obstacle

		down_bounds = Bounds(down_minimum_environment, down_maximum_environment)

		return down_bounds
		
	def collides(self, state_local_position: LocalPosition) -> bool:
		pass

	def add_obstacle(self, new_obstacle: Obstacle) -> None:
		pass

	def _update_bounds(self) -> None:
		#Should find the bounds based on the obstacle tree or list
		pass

	@property
	def north_bounds(self):
		return self._north_bounds

	@property
	def east_bounds(self):
		return self._east_bounds

	@property
	def down_bounds(self):
		return self._down_bounds

	@property
	def obstacles(self):
		return self._obstacles

	@property
	def geodetic_home(self):
		return self._geodetic_home

	@property
	def geodetic_goal(self):
		return self._geodetic_goal

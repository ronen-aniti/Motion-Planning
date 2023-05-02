from halfsize import HalfSize
from local_position import LocalPosition
import numpy as np


class Obstacle:
	def __init__(self, local_position: LocalPosition, halfsize: HalfSize):
		self._local_position = local_position
		self._halfsize = halfsize
		self._safety = self._determine_safety()
		self._height = self._local_position.down + self._halfsize.down
		self.position_array = np.array([local_position.north, local_position.east, local_position.down])
		self.halfsize_array = np.array([halfsize.north, halfsize.east, halfsize.down])
		self._ground_position_array = np.array([local_position.north, local_position.east])

	def _determine_safety(self) -> float:
		safety = np.hypot(self._halfsize.north, self._halfsize.east)
		return safety

	@property
	def local_position(self):
		return self._local_position

	@property
	def halfsize(self):
		return self._halfsize

	@property
	def safety(self):
		return self._safety

	@property
	def height(self):
		return self._height

	@property
	def ground_position(self):
		return self._ground_position_array
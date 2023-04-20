class HalfSize:
	def __init__(self, north: float, east: float, down: float):
		self._north = north
		self._east = east
		self._down = down

	@property
	def north(self):
		return self._north
	
	@property
	def east(self):
		return self._east

	@property
	def down(self):
		return self._down
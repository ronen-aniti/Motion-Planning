class Bounds:
	def __init__(self, minimum: float, maximum: float):
		self._minimum = minimum
		self._maximum = maximum

	@property
	def minimum(self):
		return self._minimum

	@property
	def maximum(self):
		return self._maximum
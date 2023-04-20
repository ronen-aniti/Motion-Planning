from geodetic_position import GeodeticPosition

class StartGoalPair:
	def __init__(self, start: GeodeticPosition, goal: GeodeticPosition):
		self._start = start
		self._goal = goal

	@property
	def start(self):
		return self._start	

	@property
	def goal(self):
		return self._goal
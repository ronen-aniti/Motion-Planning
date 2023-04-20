class GeodeticPosition:
	def __init__(self, longitude: float, latitude: float, altitude: float):
		self._longitude = longitude
		self._latitude = latitude
		self._altitude = altitude

	@property
	def longitude(self):
		return self._longitude

	@property
	def latitude(self):
		return self._latitude

	@property
	def altitude(self):
		return self._altitude
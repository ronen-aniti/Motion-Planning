
import numpy as np
import utm

class LocalPosition:
	def __init__(self, north: float, east: float, down: float):
		self._north = north
		self._east = east
		self._down = down

	def convert_to_geodetic_from_local(self, geodetic_home: "GeodeticPosition") -> "GeodeticPosition":
		from geodetic_position import GeodeticPosition # This import helps to avoid circular import issues
		geodetic_home_east, geodetic_home_north, geodetic_home_zone, geodetic_home_zone_letter = utm.from_latlon(geodetic_home.latitude, geodetic_home.longitude)
		latitude, longitude = utm.to_latlon(geodetic_home_east + self._east, geodetic_home_north + self._north, geodetic_home_zone, geodetic_home_zone_letter)
		altitude = self._down - geodetic_home.altitude # self._down should be positive, even though it's called "down".
		geodetic_position = GeodeticPosition(longitude, latitude, altitude)
		return geodetic_position

	@property
	def north(self):
		return self._north
	
	@property
	def east(self):
		return self._east
	@property
	def down(self):
		return self._down
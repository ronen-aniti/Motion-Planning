from local_position import LocalPosition
import numpy as np
import utm

class GeodeticPosition:
	def __init__(self, longitude: float, latitude: float, altitude: float):
		self._longitude = longitude
		self._latitude = latitude
		self._altitude = altitude

	def local_relative_to(self, geodetic_reference_position: 'GeodeticPosition') -> 'LocalPosition':
		home_east, home_north, home_zone, home_letter = utm.from_latlon(geodetic_reference_position.latitude, geodetic_reference_position.longitude)
		current_east, current_north, current_zone, current_letter = utm.from_latlon(self._latitude, self._longitude)
		local_position_relative_to_home = LocalPosition(current_north - home_north, current_east - home_east, self._altitude - geodetic_reference_position.altitude)
		return local_position_relative_to_home

	@property
	def longitude(self):
		return self._longitude

	@property
	def latitude(self):
		return self._latitude

	@property
	def altitude(self):
		return self._altitude




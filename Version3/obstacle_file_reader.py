import csv
from geodetic_position import GeodeticPosition
import numpy as np

class ObstacleFileReader:
	def __init__(self, filename: str):
		self._filename = filename

	def extract_geodetic_home(self) -> GeodeticPosition:
		with open(self._filename) as file:
			reader = csv.reader(file)
			first_line = next(reader)
		home_longitude = float(first_line[1].split(' lon0 ')[1])
		home_latitude = float(first_line[0].split('lat0 ')[1])
		home_altitude = 0.0 # Assume home altitude is 0.0 meters above ground
		geodetic_home = GeodeticPosition(home_longitude, home_latitude, home_altitude)

		return geodetic_home

	def extract_obstacles_as_array(self) -> np.ndarray:
		obstacle_array = np.loadtxt(self._filename, delimiter=',', skiprows=2)

		return obstacle_array
		
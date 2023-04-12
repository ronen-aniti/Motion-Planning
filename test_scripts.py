import unittest
from planning_utils import calculate_nearest_free_cell_in_2d, build_map_and_take_measurements, read_destinations, Actions, a_star, remove_collinear, get_user_planning_scheme
import numpy as np
import csv
import pdb

class TestPlanningUtils(unittest.TestCase):
	def test_collides(self):
		elevation_map, ned_boundaries, map_size = build_map_and_take_measurements('colliders.csv')
		destinations = read_destinations('destinations.json')
		self.assertEqual(collides(elevation_map, 0, 0, 10), True)
	def test_calculate_nearest_free_cell_in_2d(self):
		elevation_map, ned_boundaries, map_size = build_map_and_take_measurements('colliders.csv')
		northing_index, easting_index, intermediate_goal_altitude = calculate_nearest_free_cell_in_2d(elevation_map, 0, 0, 10)
		self.assertEqual(northing_index, 51)
		self.assertEqual(easting_index, 0)
		self.assertEqual(elevation_map[51][0], 0.0)
	def test_Actions(self):
		self.assertEqual(Actions.NORTH.delta, (1, 0))
	def test_a_star(self):
		elevation_map, ned_boundaries, map_size = build_map_and_take_measurements('colliders.csv')
		path, cost = a_star(elevation_map, (51, 0), (61, 10), 10, euclidean_distance)
		expected_path = [(51, 0), (52, 1), (53, 2), (54, 3), (55, 4), (56, 5), (57, 6), (58, 7), (59, 8), (60, 9), (61, 10)]
		expected_cost = 10 * np.sqrt(2)
		self.assertEqual(path, expected_path)
		self.assertAlmostEqual(cost, expected_cost)
	def test_remove_collinear(self):
		path = [(100, 1), (101, 1), (102, 1), (102, 2)]
		expected_result = [(100, 1), (102, 1), (102, 2)]
		self.assertEqual(expected_result, remove_collinear(path))
	def test_get_user_planning_scheme(self):
		get_user_planning_scheme()


	




if __name__ == '__main__':
	unittest.main()

import unittest
from planning_utils import calculate_nearest_free_cell_in_2d, collides, build_map_and_take_measurements, read_destinations


class TestPlanningUtils(unittest.TestCase):
	def test_collides(self):
		elevation_map, ned_boundaries, map_size = build_map_and_take_measurements('colliders.csv')
		destinations = read_destinations('destinations.json')
		self.assertEqual(collides(elevation_map, 0, 0, 10), True)

if __name__ == '__main__':
	unittest.main()

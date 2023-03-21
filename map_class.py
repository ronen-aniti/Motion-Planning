import numpy as np
from typing import List, Tuple

class Map:
    """
    The general map class, from which specific map representations are derived
    """
    def __init__(self, filename: str, title: str, safety: int, global_home: Tuple[float, float, float], current_local_position: Tuple[float, float, float], goal_local_position: Tuple[float, float, float]):
        self.filename = filename
        self.title = title
        self.safety = safety
        self.global_home = global_home
        self.current_local_position = current_local_position
        self.current_altitude = self.current_local_position[2]
        self.goal_local_position = goal_local_position
        self.goal_altitude = self.goal_local_position[2]
        self.data = np.loadtxt(self.filename, delimiter=',', skiprows=2)
        self.ned_boundaries, self.map_size = self.take_measurements()
        self.elevation_map = self.compute_elevation_map()
        self.current_grid_location = self.determine_grid_location(self.current_local_position)
        self.goal_grid_location = self.determine_grid_location(self.goal_local_position)

    def determine_grid_location(self, local_position: Tuple[float, float, float]) -> Tuple[int, int]:
        """
        Determine current grid location given a local position
        """

        # First, unpack the northing and easting deltas from current local position
        northings, eastings, _ = local_position

        # Second, determine the grid north and east coordinates.
        ## Subtract away the grid's northing and easting offsets in order to do so.
        grid_north = int(northings - self.ned_boundaries[0])
        grid_east = int(eastings - self.ned_boundaries[2])
        
        # Return the drone's current grid position
        return (grid_north, grid_east)



    def take_measurements(self) -> Tuple[List[int], List[int]]:
        """
        Scans the csv map file and returns the NED boundaries relative to global home
        """
        
        # First, calculate NED boundaries: North min, North max, East min, East Max, Alt min, Alt max.
        ned_boundaries = [
            int(np.floor(np.amin(self.data[:,0] - self.data[:,3])) - self.safety), 
            int(np.ceil(np.amax(self.data[:,0] + self.data[:,3])) + self.safety),
            int(np.floor(np.amin(self.data[:,1] - self.data[:,4])) - self.safety), 
            int(np.ceil(np.amax(self.data[:,1] + self.data[:,4])) + self.safety),
            0, 
            int(np.ceil(np.amax(self.data[:,2] + self.data[:,5])) + self.safety)
            ]

        # Second, calculate the size of the map
        map_size = [
            ned_boundaries[1] - ned_boundaries[0],
            ned_boundaries[3] - ned_boundaries[2],
            ned_boundaries[5] - ned_boundaries[4]
        ]

        # Third, return the calculated quantities
        return ned_boundaries, map_size

    def compute_elevation_map(self) -> np.ndarray:
        """
        Compute a '2.5d' map of the drone's environment
        """

        # First, initialize a grid of zeros
        elevation_map = np.zeros((self.map_size[0], self.map_size[1]))
        
        # Second, build a 2.5d grid representation of the drone's environment
        ## Iterate through the map data file to do this
        for i in range(self.data.shape[0]):
            north, east, down, d_north, d_east, d_down = self.data[i, :]
            height = down + d_down
            obstacle_boundaries = [
                int(north - self.ned_boundaries[0] - d_north - self.safety),
                int(north - self.ned_boundaries[0] + d_north + self.safety),
                int(east - self.ned_boundaries[2] - d_east - self.safety),
                int(east - self.ned_boundaries[2] + d_east + self.safety)
            ]
            elevation_map[obstacle_boundaries[0] : obstacle_boundaries[1]+1, obstacle_boundaries[2] : obstacle_boundaries[2] : obstacle_boundaries[3]+1] = height - self.ned_boundaries[4]

        # Third, return the 2.5d map
        return elevation_map

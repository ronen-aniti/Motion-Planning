from map_class import Map
import numpy as np
import matplotlib.pyplot as plt
from skimage.morphology import medial_axis
from skimage.util import invert
from queue import PriorityQueue
from collections import deque
import pdb


class MedialAxisGridMap(Map):
    def __init__(self, filename, title, safety, global_home, current_local_position, goal_local_position):
        super().__init__(filename, title, safety, global_home, current_local_position, goal_local_position)
        grid = self.compute_grid()
        self.grid = medial_axis(invert(grid))
        

    def compute_grid(self):
        """
        Build a grid to represent the drone's environment
        """
        
        # First, initialize a grid of zeros
        grid = np.zeros((self.map_size[0], self.map_size[1]), dtype='float64')
        
        # Second, build a grid representation of the drone's environment
        obstacle_boundaries = [0, 0, 0, 0]
        ## Iterate through the map data file to do this
        for i in range(self.data.shape[0]):
            north, east, down, d_north, d_east, d_down = self.data[i, :]
            if (down + d_down) + self.safety > self.goal_altitude:
                obstacle_boundaries = [
                    int(north - self.ned_boundaries[0] - d_north - self.safety),
                    int(north - self.ned_boundaries[0] + d_north + self.safety),
                    int(east - self.ned_boundaries[2] - d_east - self.safety),
                    int(east - self.ned_boundaries[2] + d_east + self.safety)
                ]
                grid[obstacle_boundaries[0] : obstacle_boundaries[1]+1, obstacle_boundaries[2] : obstacle_boundaries[3]+1] = 1.0

        # Third, return the grid map
        return grid

    def search_grid(self):
        """
        Generates a position command sequence that will bring the drone from start to goal
        """
        current_local_position = self.current_local_position
        current_local_position[2] = self.goal_altitude # The first waypoint will be at the target altitude
        self.current_grid_location = self.find_closest(self.current_grid_location)
        self.goal_grid_location = self.find_closest(self.goal_grid_location)

        # Second, begin an A* search routine
        frontier = PriorityQueue()
        visited = set()
        frontier.put((0.0, self.current_grid_location))
        visited.add(self.current_grid_location)
        travel_cost = 0.0
        grid_sequence = []
        found = False
        pathinfo = {}

        ## Repeat this subroutine until a path from start to goal is found
        while not frontier.empty():
            
            _, gridcell = frontier.get()

            if gridcell == self.goal_grid_location:
                found = True
                print('Found a path.')
                break

            free_neighbors = self.explore_free_neighbors(gridcell)
        
            for free_neighbor in free_neighbors:
                candidate_cell = free_neighbor[0]
                candidate_north, candidate_east = candidate_cell
                heuristic_cost = np.sqrt((candidate_north - self.goal_grid_location[0])**2 + (candidate_east - self.goal_grid_location[1])**2)
                action_cost = free_neighbor[1] 
                incremental_cost = action_cost + heuristic_cost
                candidate_total_cost = travel_cost + incremental_cost
                if candidate_cell not in visited:
                    frontier.put((candidate_total_cost, candidate_cell))
                    pathinfo[candidate_cell] = (gridcell, action_cost)
                    visited.add(candidate_cell)
        
        # Third, once the goal gridcell has been found, generate a sequence of gricells from start to goal
        if found:
            subgoal = self.goal_grid_location
            origin, action_cost = pathinfo[subgoal]
            grid_sequence.append(subgoal)
            while origin != self.current_grid_location:
                subgoal = origin
                origin, action_cost = pathinfo[subgoal]
                grid_sequence.append(subgoal)
            grid_sequence.append(origin)
            grid_sequence = grid_sequence[::-1]

            
            
            ## Remove collinear gridcells from the waypoint sequence
            grid_sequence = self.remove_collinear(grid_sequence)

            ## Plot the updated grid sequence
            #self.plot_path(grid_sequence)

            ## Covert the grid sequence into a waypoint sequence
            waypoint_commands = [self.grid_to_waypoint(gridcell) for gridcell in grid_sequence]
            
            ## Return the waypoint sequence
            return waypoint_commands

        else:
            raise Exception('Failed to find a path.')

    
    def remove_collinear(self, grid_sequence):
        """
        Removes collinear gridcells
        """
        culled_grid_sequence = [grid_sequence[0], grid_sequence[1]]
        for i in range(1, len(grid_sequence) - 1):
            x1, y1 = grid_sequence[i - 1]
            x2, y2 = grid_sequence[i]
            x3, y3 = grid_sequence[i + 1]

            collinear = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2) == 0

            if not collinear:
                waypoint_pair.append(grid_sequence[i + 1])


        return culled_grid_sequence

    def find_closest(self, gridcell):
        """
        Finds the closest free gridcell in a 2d grid
        """
        x, y = gridcell
        rows, cols = len(self.grid), len(self.grid[0])
        visited = set((x, y))
        queue = deque([(x, y, 0)])

        while queue:
            x, y, distance = queue.popleft()
            if self.grid[x][y] == 1.0:
                return (x, y)

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < rows and 0 <= ny < cols and (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny, distance + 1))


    def grid_to_waypoint(self, gridcell):
        """
        Converts a grid cell into a waypoint command
        """
        waypoint = [[gridcell[0] + self.ned_boundaries[0], gridcell[1] + self.ned_boundaries[2], self.goal_altitude, 0]]

        return waypoint



    def explore_free_neighbors(self, gridcell):
        """
        Returns a gridcell's free neighbors along with the cost of getting to each
        """
        
        # First, define the action set
        actions = {
            'LEFT': (0, -1, 1),
            'RIGHT': (0, 1, 1),
            'UP': (-1, 0, 1),
            'DOWN': (1, 0, 1),
            'NORTHEAST': (-1, 1, np.sqrt(2)),
            'NORTHWEST': (1, 1, np.sqrt(2)),
            'SOUTHEAST': (1, 1, np.sqrt(2)),
            'SOUTHWEST': (1, -1, np.sqrt(2))
        }
        
        grid_north, grid_east = gridcell

        rows, cols = self.grid.shape

        free_neighbors = [
            ((grid_north + north_delta, grid_east + east_delta), action_cost)
            for north_delta, east_delta, action_cost in actions.values()
            if 0 <= grid_north + north_delta < rows and 0 <= grid_east + east_delta < cols and self.grid[grid_north + north_delta, grid_east + east_delta] == True
        ]

        return free_neighbors

    def plot_grid(self):
        plt.imshow(self.grid, origin='lower', cmap='Greys')
        plt.title(self.title)
        plt.xlabel('Eastings (m)')
        plt.ylabel('Northings (m)')
        plt.show()

    def plot_path(self, grid_sequence):
        """
        Plots the path from start to goal
        """
        grid_sequence_north = [grid_cell[0] for grid_cell in grid_sequence]
        grid_sequence_east = [grid_cell[1] for grid_cell in grid_sequence]
        
        plt.imshow(self.grid, origin='lower', cmap='Greys')
        plt.plot(grid_sequence_east, grid_sequence_north, color='blue', linestyle='-', marker='o')
        plt.title(self.title)
        plt.xlabel('Eastings (m)')
        plt.ylabel('Northings (m)')
        plt.show()
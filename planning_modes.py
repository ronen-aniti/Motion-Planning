from enum import Enum, auto

class PlanningModes(Enum):
    GRID2D = 1
    MEDAXIS = 2
    POTFIELD = 3
    VOXEL = 4
    VORONOI = 5
    PRM = 6
    RRT = 7

    def __str__(self):
        if self == PlanningModes.GRID2D:
            return "2D grid and A* search"
        elif self == PlanningModes.MEDAXIS:
            return "Medial Axis grid with A* search"
        elif self == PlanningModes.POTFIELD:
            return "Potential field"
        elif self == PlanningModes.VOXEL:
            return "Voxel map with A* search"
        elif self == PlanningModes.VORONOI: 
            return "Voronoi graph with A*"
        elif self == PlanningModes.PRM:
            return "Probabilistic Road Map (PRM)"
        elif self == PlanningModes.RRT:
            return "Rapidly-Exploring Random Tree (RRT)"
import csv
import numpy as np
import utm
from sklearn.neighbors import KDTree
import random 
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.patches import Polygon
import networkx as nx
import itertools
import pdb



"""
Driver code
0. Read or save a goal geo object
1. Create environment object from colliders file. This object will include attributes
   containing a list of obstacle objects, a localbounds object, a geodetic home object, 
   a safety_radius
2. 
"""
class Environment:
	"""
	Given colliders data file.
	Creates obstacle object for each row of the table.
	Stores a list or kd tree of obtacles in an attribute.
	Creates a geodetic object for home, stores in attribute
	Creates a geodetic object for global goal, stores in attribute.


	attributes:
	obstacles->list or kdtree of obstacle objects
	geo home->geodetic object
	local bounds -> bounds object
	minimum safe radius for obstacle proximity checking (maximum ground radius of all the obstacle objects) -> float
	goal geodetic object

	private methods:
	read raw global home data -> geo object
	read raw obstacle data -> array
	add an obstacle object -> obstacle object
	create obstacle list of objects -> list
	create obstacle tree of objects -> kdtree
	determine bounds -> bounds object
	determine safety radius -> float
	

	"""
	pass


class Obstacle:
	"""
	Given center point local object, 3 numbers
	Assigns center point attribute and an attribute for each half size
		and an attribute for the  of the north/east half size ("ground_radius")

	attributes:
	local->north, east, down
	halfsizes->dn, de, dd

	
	"""
	pass

class PathPlanner:
	"""
	A sequence of states. 
	Given a start state, a goal state, and a choice of search algorithm
		(state 1, state 2, and then either ellipsoid or global plan using sampling techniques.)
	Computes the sequence of states (S1, S2, ..., Sn)
	Has methods to assist in searching that accept environment attributes as arguments and return state sequences.
	
	Attributes
	start_state
	goal_state
	middle_states

	examples:

	path_planner_object.search(s1, s2, environment, 'local')
	takes samples (sample states).
	builds a connected graph.
	searches for the lowest cost sequence

	-> runs a private method to conduct A* search, using 

	path_planner_object.search(s1, s2, 'global')


	"""
	pass

class LocalPosition:
	"""
	Given geodetic object and geodetic object for home
	Assigns north, east, down

	attributes:
	north
	east
	down
	"""
	pass

class GeodeticPosition:
	"""
	Given 3 numbers
	Assigns lon, lat, and alt.

	attributes:
	longitude
	latitude
	altitude
	"""
	pass

class State:
	"""
	Given: geodetic object, env

	Computes local object
	Assigns heading = 0

	Computes potential given environment.obstacles (object array or tree)
	by using +k2/dg^2 and -k1/do^2 with nearest neighbor obstacle and distance to environment.global_goal (geodetic object).
	

	attributes
	geodetic object
	local object
	geodetic object (home)
	heading float
	distance_to_nearest_obstacle
	distance_to_global_goal
	cost (k1/dg^2 - k2/do^2)

	private methods:
	find do given env
	find dg given env
	computes local object given env, geo
	

	"""
	pass


class Ellipsoid:
	"""
	Given: state1, state2
	Computes f1, f2, h, k, l, a.
	Assigns b and c.
	
	attributes:
	a = focus2-focus1
	b = const1 (maybe a/2)
	c = const1
	center (n, e, d)
	focus1 (n, e, d)
	focus2 ()
	
	methods:
	contains(state)
		return (x - h)^2 / a^2 + (y - k)^2 / b^2 + (z - l)^2 / c^2 < 1
	
	"""

class LocalBounds:

	"""
	Given 3 numbers assigns:
	Attributes:
	north
	east
	down
	"""
	pass
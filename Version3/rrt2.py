import numpy as np
import random
import matplotlib.pyplot as plt

class Node:
	def __init__(self, local_position, parent=None, cost=0):
		self.local_position = local_position
		self.parent_node = parent_node
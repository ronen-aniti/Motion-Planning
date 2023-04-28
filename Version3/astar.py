import numpy as np
from queue import PriorityQueue
from sklearn.neighbors import KDTree
import pdb
import matplotlib.pyplot as plt
import random

class Node:
	def __init__(self, value):
		self.value = value
		self.g_score = float('inf')
		self.f_score = float('inf')
		self.h_score = float('inf')
		self.parent = None

	def __str__(self):
		return f"Value: {self.value}"

class Graph:
	def __init__(self):
		self.nodes=[]
		self.edges = {}
	
	def __str__(self):
		return f"The graph's nodes contain these values: {[node.value for node in self.nodes]}"
	
	def add_node(self, node):
		self.nodes.append(node)
	
	def add_nodes_from_list(self, list_of_nodes):
		for node in list_of_nodes:
			self.nodes.append(node)
	
	def add_edge(self, node1, node2):
		computed_weight = np.linalg.norm(node2.value-node1.value)
		if node1 not in self.edges:
			self.edges[node1] = {}
		if node2 not in self.edges:
			self.edges[node2] = {}
		self.edges[node1][node2] = computed_weight
		self.edges[node2][node1] = computed_weight

	def heuristic_cost_estimate(self, node1, node2):
		return np.linalg.norm(node1.value-node2.value)

	def reconstruct_path(self, current):
		path = [current]
		while current.parent is not None:
			current = current.parent
			path.append(current)
		return path[::-1]

	def astar(self, start_node, goal_node):
		open_set = PriorityQueue()
		open_set.put((0, start_node))
		start_node.g_score = 0
		start_node.f_score = start_node.g_score + self.heuristic_cost_estimate(start_node, goal_node) # f = g + h

		while not open_set.empty():
			current = open_set.get()[1]

			if current == goal_node:
				return self.reconstruct_path(current)

			for neighbor in self.edges[current]:
				tentative_g_score = current.g_score + self.edges[current][neighbor]
				if tentative_g_score < neighbor.g_score:
					neighbor.parent = current
					neighbor.g_score = tentative_g_score
					neighbor.f_score = tentative_g_score + self.heuristic_cost_estimate(neighbor, goal_node)
					if neighbor not in open_set.queue:
						open_set.put((neighbor.f_score, neighbor))
		return None

	def plot_graph_and_path(self, start_node, goal_node):
			fig, ax = plt.subplots()
			ax.set_aspect('equal')

			# plot nodes
			for node in self.nodes:
				ax.scatter(node.value[0], node.value[1], marker='o', color='red')

			# plot graph edges as blue lines
			for node1 in self.edges:
				for node2 in self.edges[node1]:
					x1, y1 = node1.value
					x2, y2 = node2.value
					ax.plot([x1, x2], [y1, y2], 'b', linewidth=0.25)

			# plot path edges as green lines
			path = self.astar(start_node, goal_node)
			if path is not None:
				for i in range(len(path)-1):
					node1 = path[i]
					node2 = path[i+1]
					x1, y1 = node1.value
					x2, y2 = node2.value
					ax.plot([x1, x2], [y1, y2], 'g', linewidth=5, alpha=0.5)

			# plot start and goal nodes with labels
			ax.plot(start_node.value[0], start_node.value[1], 'go')
			ax.text(start_node.value[0]+0.1, start_node.value[1]+0.1, 'Start')
			ax.plot(goal_node.value[0], goal_node.value[1], 'yo')
			ax.text(goal_node.value[0]+0.1, goal_node.value[1]+0.1, 'Goal')

			plt.show()

	def add_n_random_nodes(self, n):
		num_samples=n
		for _ in range(num_samples):
			x=np.random.uniform(0,100)
			y=np.random.uniform(0,100)
			node = Node(np.array([x,y]))
			graph.add_node(node)
		num_edges=num_samples*5
		for _ in range(num_edges):
			graph.add_edge(random.choice(graph.nodes), random.choice(graph.nodes))

"""
graph = Graph()
graph.add_n_random_nodes(50)
graph.plot_graph_and_path(random.choice(graph.nodes), random.choice(graph.nodes))



"""
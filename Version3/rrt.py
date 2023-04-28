import numpy as np
import random
import matplotlib.pyplot as plt

class RRTNode:
	def __init__(self, position, parent=None, cost=0):
		self.position = position
		self.parent = parent
		self.cost = cost

class RRT:
	def __init__(self, start, goal, step_size, max_iter, space_bounds):
		self.start = RRTNode(np.array(start))  # convert start to RRTNode
		self.goal = np.array(goal)
		self.step_size = step_size
		self.max_iter = max_iter
		self.nodes = [self.start]  # include start node in nodes list
		self.space_bounds = space_bounds
		self.goal_node = None

	def get_nearest_node(self, position):
		position = np.array(position)
		if len(self.nodes) == 0:
			return None
		distances = [np.linalg.norm(node.position - position) for node in self.nodes]
		nearest_node = self.nodes[np.array(distances).argmin()]
		return nearest_node

	def generate_random_point(self):
		x = random.uniform(self.space_bounds[0][0], self.space_bounds[0][1])
		y = random.uniform(self.space_bounds[1][0], self.space_bounds[1][1])
		return (x, y)

	def step_from_to(self, start_node, end):
		start = start_node.position
		end = np.array(end)
		distance = np.linalg.norm(end - start)
		if distance < self.step_size:
			return RRTNode(end, parent=start_node)
		else:
			unit_vector = (end - start) / distance
			new_node_position = start + self.step_size * unit_vector
			return RRTNode(new_node_position, parent=start_node)

	def is_collision_free(self, start_node, end_node):
		start = start_node.position
		end = end_node.position
		delta = end - start
		distance = np.linalg.norm(delta)
		unit_vector = delta / distance
		num_steps = int(distance / self.step_size)
		for i in range(num_steps):
			sub_segment_start = start + i * self.step_size * unit_vector
			sub_segment_end = start + (i + 1) * self.step_size * unit_vector
			if self.is_segment_in_collision(sub_segment_start, sub_segment_end):
				return False
		return True

	def is_segment_in_collision(self, subsegment_start, subsegment_end):
		return False

	def find_path(self):
		min_goal_distance = float('inf')
		for i in range(self.max_iter):
			random_point = self.generate_random_point()
			nearest_node = self.get_nearest_node(random_point)
			if nearest_node is None:
				continue
			new_node = self.step_from_to(nearest_node, random_point)
			new_node.cost = nearest_node.cost + self.step_size
			if self.is_collision_free(nearest_node, new_node):
				self.nodes.append(new_node)
				goal_distance = np.linalg.norm(new_node.position - self.goal)
				if goal_distance < min_goal_distance:
					self.goal_node = new_node
					min_goal_distance = goal_distance
		return self.construct_path(self.goal_node) if self.goal_node is not None else None

	def construct_path(self, goal_node):
		current_node = goal_node
		path = [current_node]
		while current_node.parent is not None:
			current_node = current_node.parent
			path.append(current_node)
		path.reverse()
		return path

	def run(self):
		path = self.find_path()
		if path is not None:
			print("Found a path from start to goal")
			for node in path:
				return path
		else:
			print("Failed to find a path from start to goal")
			return None

			
	def plot_tree(self):
		fig, ax = plt.subplots()

		# Plot the start and goal nodes
		ax.scatter(self.start.position[0], self.start.position[1], c='g', marker='o', label='start')
		ax.scatter(self.goal[0], self.goal[1], c='r', marker='o', label='goal')

		# Plot each node in the RRT and its parent
		for node in self.nodes:
			if node.parent is not None:
				ax.plot([node.parent.position[0], node.position[0]], [node.parent.position[1], node.position[1]], c='b', linewidth=0.5)

				# Uncomment the line below to plot the nodes as small blue dots
				ax.scatter(node.position[0], node.position[1], c='b', marker='.', s=10)

		# Set the x and y limits of the plot
		ax.set_xlim(self.space_bounds[0][0], self.space_bounds[0][1])
		ax.set_ylim(self.space_bounds[1][0], self.space_bounds[1][1])

		# Add labels and legend to the plot
		ax.set_xlabel('X')
		ax.set_ylabel('Y')
		ax.set_title('RRT Tree')
		ax.legend()

		

	def plot_path(self, path):
		if path is not None:
			for i in range(len(path) - 1):
				start_node = path[i]
				end_node = path[i + 1]
				plt.plot([start_node.position[0], end_node.position[0]], [start_node.position[1], end_node.position[1]], c='m', linewidth=2.0, linestyle='-', label='Path' if i == 0 else "")

rrt = RRT((5, 5), (140, 150), 5, 1000, [[-50, 200], [-50, 200]])
path = rrt.run()
rrt.plot_tree()
rrt.plot_path(path)
plt.show()

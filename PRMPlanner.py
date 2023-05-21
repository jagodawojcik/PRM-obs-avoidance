import numpy as np
import math
import matplotlib.pyplot as plt

#parameters for PRM
N_NODES = 150
RADIUS = 5

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.path_x = []
        self.path_y = []
        self.neighbors = []
        self.parent = None 
        self.visited = False
        self.dist = float('inf')

class PRM:
    def __init__(self, start,
                 goal,
                 obstacle_list,
                 rand_area,
                 robot_radius,
                 expand_dis = 3.0,
                 path_resolution = 0.5,
                 num_nodes=N_NODES,
                 connect_radius = RADIUS):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.obstacle_list = obstacle_list
        self.robot_radius = robot_radius
        self.x_min = rand_area[0]
        self.x_max = rand_area[1]
        self.y_min = rand_area[0]
        self.y_max = rand_area[1]
        self.num_nodes = num_nodes
        self.connect_radius = connect_radius
        self.node_list = []
        self.path = []

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

    def steer(self, from_node: Node, to_node: Node,
              extend_length=float("inf")):

        new_node = Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_roadmap(self):
        # Generate random nodes
        i = 0
        while i <= self.num_nodes:
            x = np.random.uniform(self.x_min, self.x_max)
            y = np.random.uniform(self.y_min, self.y_max)
            node = Node(x, y)
            if not self.check_samples(x, y, self.obstacle_list):
                self.node_list.append(node)
                i = i+1

        # Add start and goal nodes to the graph
        self.node_list.append(self.start)
        self.node_list.append(self.goal)

        # Connect nodes that are within a certain distance of each other and have no obstacles between them
        for i, node in enumerate(self.node_list):
            distances = np.sqrt((node.x - np.array([n.x for n in self.node_list]))**2 + (node.y - np.array([n.y for n in self.node_list]))**2)
            neighbors_idx = np.where((distances <= self.connect_radius) & (distances > 0))[0]
            for j in neighbors_idx:
                neighbor = self.node_list[j]
                neighbor_pathed = self.steer(node, neighbor, self.expand_dis)
                if self.check_collision(neighbor_pathed, self.obstacle_list, self.robot_radius):
                    node.neighbors.append(neighbor)
                    neighbor.neighbors.append(node)


    def solve(self):
        # Run Dijkstra's algorithm to find the shortest path
        self.start.dist = 0
        unvisited_nodes = set(self.node_list)

        while unvisited_nodes:
            # Find the node with the smallest distance from the start
            current_node = min(unvisited_nodes, key=lambda node: node.dist)
            current_node.visited = True
            unvisited_nodes.remove(current_node)

            # Update the distances of the neighbors
            for neighbor in current_node.neighbors:
                if not neighbor.visited:
                    dist = current_node.dist + np.sqrt((neighbor.x - current_node.x)**2 + (neighbor.y - current_node.y)**2)
                    if dist < neighbor.dist:
                        neighbor.dist = dist
                        neighbor.parent = current_node


        # Reconstruct the path from the start to the goal
        if self.goal.dist == float('inf'):
            return

        self.path = [self.goal]
        while self.path[-1].parent != None:
            self.path.append(self.path[-1].parent)

        self.path.append(self.start)
        self.path.reverse()

        return self.path

    def planning(self):
        # Generate the roadmap
        self.generate_roadmap()

        # Solve the problem using Dijkstra's algorithm
        path = self.solve()
        return [(v.x, v.y) for v in path] if path is not None else None

    @staticmethod
    def check_collision(node, obstacleList, robot_radius):

        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= (size + robot_radius)**2:
                return False  # collision

        return True  # safe

    def line_intersect_circle(self,x1, y1, x2, y2, cx, cy, r):
        # Compute the distance between the circle center and the line endpoints
        dx1 = cx - x1
        dy1 = cy - y1
        dx2 = cx - x2
        dy2 = cy - y2

        # Compute the length of the line segment
        length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        # Compute the dot product between the line vector and the vector from the first endpoint to the circle center
        dot = dx1 * (x2 - x1) + dy1 * (y2 - y1)

        # Compute the closest point on the line to the circle center
        if length == 0:
            closest_x = x1
            closest_y = y1
        else:
            closest_x = x1 + (dot / length ** 2) * (x2 - x1)
            closest_y = y1 + (dot / length ** 2) * (y2 - y1)

        # Check if the closest point is inside the line segment
        if np.sqrt((closest_x - x1) ** 2 + (closest_y - y1) ** 2) <= length and np.sqrt((closest_x - x2) ** 2 + (closest_y - y2) ** 2) <= length:
            # Check if the closest point is inside the circle
            if np.sqrt((closest_x - cx) ** 2 + (closest_y - cy) ** 2) <= r:
                return True

        # If we get here, the line does not intersect the circle
        return False

    def check_samples(self, x, y, circles):

    #Check if a given point (x, y) falls within any of the circles defined by their (cx, cy, r) coordinates.
        for cx, cy, r in circles:
            if (x - cx) ** 2 + (y - cy) ** 2 <= (r + self.robot_radius) ** 2:
                return True
        return False

    def plot(self):
        # Plot the graph and path
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.set_xlim([self.x_min, self.x_max])
        ax.set_ylim([self.y_min, self.y_max])

        # Plot obstacles
        for obstacle in self.obstacle_list:
            circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[2], color='r', fill = False)
            ax.add_patch(circle)

        # Plot nodes and edges
        for node in self.node_list:
            for neighbor in node.neighbors:
                plt.plot([node.x, neighbor.x], [node.y, neighbor.y], 'k-', alpha=0.3)
            plt.plot(node.x, node.y, 'bo', markersize=2)

        # Plot start and goal nodes
        plt.plot(self.start.x, self.start.y, 'gx', markersize=3)
        plt.plot(self.goal.x, self.goal.y, 'gx', markersize=3)

        # Plot path
        if self.path:
            x = [node.x for node in self.path]
            y = [node.y for node in self.path]
            plt.plot(x, y, 'g-', linewidth=3)

def main():
    obstacle_list = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
                    (9, 5, 2), (8, 10, 1)]

    # Create an instance of the PRM class
    prm = PRM((6, 10), (0, 0), obstacle_list, (-2, 16), 0.8)

    path = prm.planning()

    # Plot the graph and path
    if path is not None:
        prm.plot()
        plt.show()

if __name__ == '__main__':
    main()

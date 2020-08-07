from grid import Node, NodeGrid
from math import inf
import heapq


class PathPlanner(object):
    """
    Represents a path planner, which may use Dijkstra, Greedy Search or A* to plan a path.
    """
    def __init__(self, cost_map):
        """
        Creates a new path planner for a given cost map.

        :param cost_map: cost used in this path planner.
        :type cost_map: CostMap.
        """
        self.cost_map = cost_map
        self.node_grid = NodeGrid(cost_map)

    @staticmethod
    def construct_path(goal_node):
        """
        Extracts the path after a planning was executed.

        :param goal_node: node of the grid where the goal was found.
        :type goal_node: Node.
        :return: the path as a sequence of (x, y) positions: [(x1,y1),(x2,y2),(x3,y3),...,(xn,yn)].
        :rtype: list of tuples.
        """
        node = goal_node
        # Since we are going from the goal node to the start node following the parents, we
        # are transversing the path in reverse
        reversed_path = []
        while node is not None:
            reversed_path.append(node.get_position())
            node = node.parent
        return reversed_path[::-1]  # This syntax creates the reverse list

    def dijkstra(self, start_position, goal_position):
        """
        Plans a path using the Dijkstra algorithm.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        # Todo: implement the Dijkstra algorithm
        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path

        self.node_grid.reset()

        node = self.node_grid.get_node(*start_position)
        node.g = 0
        pq = []
        heapq.heappush(pq, (node.g, node))

        while len(pq) != 0:
            node.g, node = heapq.heappop(pq)
            if node.get_position() == goal_position:
                break
            node.closed = True
            for successor in self.node_grid.get_successors(*node.get_position()):
                successor_node = self.node_grid.get_node(*successor)
                if successor_node.closed == False:
                    if successor_node.g > node.g + self.cost_map.get_edge_cost(node.get_position(), successor):
                        successor_node.g = node.g + self.cost_map.get_edge_cost(node.get_position(), successor)
                        successor_node.parent = node
                        heapq.heappush(pq, (successor_node.g, successor_node))

        return self.construct_path(node), node.g

    def greedy(self, start_position, goal_position):
        """
        Plans a path using greedy search.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        # Todo: implement the Greedy Search algorithm
        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path
        self.node_grid.reset()

        node = self.node_grid.get_node(*start_position)
        successor_node = self.node_grid.get_node(*start_position)
        node.g = 0
        pq = []
        heapq.heappush(pq, (node.distance_to(*goal_position), node))

        out = False
        while len(pq) != 0 and out == False:
            h, node = heapq.heappop(pq)
            if node.get_position() == goal_position:
                break
            node.closed = True
            for successor in self.node_grid.get_successors(*node.get_position()):
                successor_node = self.node_grid.get_node(*successor)
                successor_node.g = node.g + self.cost_map.get_edge_cost(node.get_position(), successor)
                if successor_node.closed == False:
                    successor_node.parent = node
                    if successor_node.get_position == goal_position:
                        out = True
                        break
                    heapq.heappush(pq, (successor_node.distance_to(*goal_position), successor_node))

        return self.construct_path(successor_node), successor_node.g

    def a_star(self, start_position, goal_position):
        """
        Plans a path using A*.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        # Todo: implement the A* algorithm
        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path
        self.node_grid.reset()

        node = self.node_grid.get_node(*start_position)
        successor_node = self.node_grid.get_node(*start_position)
        node.g = 0
        node.f = node.distance_to(*goal_position)
        pq = []
        heapq.heappush(pq, (node.f, node))

        while len(pq) != 0:
            h, node = heapq.heappop(pq)
            if node.get_position() == goal_position:
                break
            node.closed = True
            for successor in self.node_grid.get_successors(*node.get_position()):
                successor_node = self.node_grid.get_node(*successor)
                if successor_node.f > node.g + self.cost_map.get_edge_cost(node.get_position(), successor) + successor_node.distance_to(*goal_position):
                    successor_node.g = node.g + self.cost_map.get_edge_cost(node.get_position(), successor)
                    successor_node.f = successor_node.g + successor_node.distance_to(*goal_position)
                    successor_node.parent = node
                    heapq.heappush(pq, (successor_node.f, successor_node))

        return self.construct_path(successor_node), successor_node.f

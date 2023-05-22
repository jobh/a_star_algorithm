import collections
import heapq


class DijkstraHeap(list):
    """
    An augmented heap for the A* algorithm. This class encapsulated the residual logic of
    the A* algorithm like for example how to manage nodes already visited that remain
    in the heap, nodes already visited that are not in the heap and from where we came to
    a visited node.

    This class will have three main elements:

        - A heap that will act as a cost queue (self).
        - A visited dict that will act as a visited set and as a mapping of the form  vertex:node
    """
    def __init__(self):
        self.visited = dict()

    def insert(self, node):
        """
        Insert a node into the Dijkstra Heap if the vertex is not already visited.

        :param node: A Node object.
        :return: None
        """

        if node.vertex not in self.visited:
            heapq.heappush(self, node)

    def pop(self):
        """
        Pop an unvisited node from the Dijkstra Heap, adding it to the visited dict.

        :return: A Node object
        """

        while self:
            next_elem = heapq.heappop(self)
            if self.visited.setdefault(next_elem.vertex, next_elem) is next_elem:
                return next_elem

    def backtrack(self, current):
        """
        Retrieve the backward path, starting from current.

        :param current: The starting vertex of the backward path
        :return: A generator of vertices; listify and reverse to get forward path
        """
        while current is not None:
            yield current
            current = self.visited[current].came_from

Node = collections.namedtuple("Node", ["cost_estimate", "cost_so_far", "vertex", "came_from"])

def a_star_search(neighbors, start, end, heuristic):
    """
    Calculates the shortest path from start to end.

    The graph is defined implicitly by a pair of functions working on vertices. The vertex representation
    may be any comparable and hashable type such as int, tuple, etc.

    :param neighbors: A function returning the (possibly directed) neighbors of a vertex, along with their costs.

        neighbors( from_vertex:V ) : Iterable( (to_vertex:V,edge_cost:float), (to_vertex:V,edge_cost:float), ...)

    :param start: The starting vertex, as type V.
    :param end: The ending vertex, as type V.
    :param heuristic: Heuristic lower-bound cost function taking arguments ( from_vertex:V, end:V ) and returning float.
    :returns: A DijkstraHeap object. The shortest path is: `reversed(list(d_heap.backtrack(end)))`.

    """

    frontier = DijkstraHeap()
    frontier.insert( Node(heuristic(start, end), 0, start, None) )

    while True:

        current_node = frontier.pop()

        if not current_node:
            raise ValueError("No path from start to end")
        if current_node.vertex == end:
            return frontier

        for neighbor, edge_cost in neighbors( current_node.vertex ):

            cost_at_neighbor = current_node.cost_so_far + edge_cost

            new_node = Node(cost_at_neighbor + heuristic(neighbor, end),
                            cost_at_neighbor,
                            neighbor,
                            current_node.vertex)

            frontier.insert(new_node)


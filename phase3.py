from graph import AdjacentVertex
from graph import Graph
import sys


class Graph2(Graph):
    def minDistance(self, distances, visited):
        """This functions returns the vertex (index) whose associated value in the dictionary distances is the smallest
         value. We only consider the set of vertices that have not been visited"""

        # Initilaize minimum distance for next node.
        min = sys.maxsize

        # Returns the vertex with minimum distance from the non-visited vertices.
        for vertex in self._vertices.keys():
            if distances[vertex] <= min and visited[vertex] == False:
                min = distances[vertex]  # update the new smallest
                min_vertex = vertex  # update the index of the smallest

        return min_vertex

    def min_number_edges(self, start: str, end: str) -> int:
        """returns the minimum number of edges from start to end.
        This function is based on the use of Dijkstra's algorithm."""

        # Create a dictionary called "visisted" whose keys are the vertices of a given graph. Initially, all
        # vertices are defined as False or not visited. When a vertex is visited, it must be marked as True.
        visited = {}
        for v in self._vertices.keys():
            visited[v] = False

        # Create a dictionary called "previous" to save the previous vertex for the key in the minimum path.
        # Initially, the previous vertex for any vertex is defined as None.
        previous = {}
        for v in self._vertices.keys():
            previous[v] = None

        # This distance will save the accumulate distance from the origin to the vertex (key).
        distances = {}
        for v in self._vertices.keys():
            distances[v] = sys.maxsize
        distances[start] = 0  # the distance from origin to itself is 0.

        for n in range(len(self._vertices)):
            # Pick the vertex with the minimum distance vertex.
            # u is always equal to origin in first iteration.
            u = self.minDistance(distances, visited)
            visited[u] = True  # mark it as true

            # For loop used to update distance value of the u's adjacent vertices. However, this is done only if
            # the current distance is greater than new distance and the vertex in not in the shotest path tree.
            # The loop visits all adjacent vertices (neighbours) for u.
            for adj in self._vertices[u]:
                i = adj.vertex
                w = adj.weight
                if visited[i] == False and distances[i] > distances[u] + w:
                    # we must update because its distance is greater than the new distance
                    distances[i] = distances[u] + w
                    previous[i] = u

        # For loop to traverse the vertices of the given graph.
        for i in self._vertices.keys():
            # Condition used to return 0 when there is no path from the origin to the indicated destination.
            if distances[i] == sys.maxsize:
                return 0
            # Condition used to return the distance once the destination is reached.
            if i == end:
                return distances[i]

    def transpose(self) -> 'Graph2':
        """returns a new graph that is the transpose graph of self"""

        # Create a new graph, which will be the tranpose of the graph.
        transpose_graph = Graph2(self._vertices, self._directed)

        # Check if the graph is directed. Otherwise, self is returned, as an undirected graph cannot be tranposed.
        if not self._directed:
            return self

        # Access the graph and add elements to the new graph (tranpose_graph).
        for i in self._vertices:
            for j in self._vertices[i]:
                transpose_graph.add_edge(j.vertex, i, j.weight)

        # Return the tranpose.
        return transpose_graph

    def dfs(self):
        """This function prints all vertices of the graph by the DFS traversal."""
        visited = {}
        for v in self._vertices.keys():
            visited[v] = False
        for v in self._vertices.keys():
            if not visited[v]:
                self._dfs(v, visited)
                return visited

    def _dfs(self, v, visited):
        """This funcion prints the DFS traversal from the vertex whose index is indexV"""
        visited[v] = True
        for adj in self._vertices[v]:
            if not visited[adj.vertex]:
                self._dfs(adj.vertex, visited)

    def is_strongly_connected(self) -> bool:
        """ This function checks if the graph is strongly connected. A directed graph is strongly connected when for any
        pair of vertices u and v, there is always a path from u to v. If the graph is undirected, the function returns
        True if the graph is connected, that is, there is a path from any vertex to any other vertex in the graph."""

        # MAIN IDEA: if every node can be accessed from a vertex u and every node can reach u, the graph is said to
        # be strongly connected. It is first determined whether all vertices are reachable from u or not. Then it is
        # checked if all the vertices can reach u (using the tranpose). If all the vertices in a transposed graph are
        # reachable from u, then all vertices in the original graph can reach u.

        # Do a DFS traversal of a given graph.
        visited = self.dfs()

        # For loop to access all vertices.
        for v in visited:
            # Condition used to know if DFS traversal visits all vertices.
            if not visited[v]:
                return False

        # Find the tranpose.
        transpose = self.transpose()

        # Do a DFD traversal of reversed graph.
        transpose_visited = transpose.dfs()

        # For loop to access all vertices.
        for v in transpose_visited:
            # Condition used to know if DFS traversal visits all vertices.
            if not transpose_visited[v]:
                return False
        return True

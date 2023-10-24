#!/usr/bin/python3


from CS312Graph import *
import time

# The class that represents a node in the priority queue
class QueueNode:
    def __init__(self, weight, vertex):
        self.weight = weight
        self.vertex = vertex

# The class that represents a priority queue implemented with an array
class ArrayQueue:
    def __init__(self):
        self.nodes = []
        self.size = 0
        self.node_index_map = {}

    # The function that inserts a node into the priority queue
    # Runtime: O(1)
    def insert(self, item):
        self.nodes.append(item)
        self.node_index_map[item.vertex] = self.size
        self.size += 1

    # The function that removes the minimum node from the priority queue
    # Runtime: O(n)
    def remove_min(self):
        min = float('inf')
        min_index = 0
        for i in range(len(self.nodes)):
            if self.nodes[i].weight < min:
                min = self.nodes[i].weight
                min_index = i
        self.node_index_map.pop(self.nodes[min_index].vertex)
        u = self.nodes.pop(min_index)
        for i in range(len(self.nodes)):
            self.node_index_map[self.nodes[i].vertex] = i
        return u

    # The function that makes the priority queue
    # Runtime: O(n)
    def make_queue(self, source_node, verticies):
        for vertex in verticies:
            if vertex == source_node:
                self.insert(QueueNode(0, vertex))
            else:
                self.insert(QueueNode(float('inf'), vertex))

    # The function that decreases the key of a node in the priority queue
    # Runtime: O(1)
    def decrease_key(self, vertex, new_weight):
        index = self.node_index_map[vertex]
        self.nodes[index].weight = new_weight



# The class that represents a priority queue implemented with a heap
class HeapQueue:
    def __init__(self, maxsize, source_node):
        self.maxsize = maxsize
        self.size = 0
        self.nodes = [0] * (self.maxsize + 1)
        self.node_index_map = {}
        self.nodes[0] = QueueNode(0, source_node)
        self.node_index_map[source_node] = 0
        self.FRONT = 0

    # The function that returns the position of the parent of a node
    # Runtime: O(1)
    def parent(self, pos):
        return pos // 2

    # The function that returns the position of the left child of a node
    # Runtime: O(1)
    def leftChild(self, pos):
        return 2 * pos

    # The function that returns the position of the right child of a node
    # Runtime: O(1)
    def rightChild(self, pos):
        return (2 * pos) + 1

    # The function that returns true if a node is a leaf node
    # Runtime: O(1)
    def isLeaf(self, pos):
        return pos*2 > self.size

    # The function that swaps two nodes in the priority queue
    # Runtime: O(1)
    def swap(self, fpos, spos):
        self.node_index_map[self.nodes[fpos].vertex], self.node_index_map[self.nodes[spos].vertex] = self.node_index_map[self.nodes[spos].vertex], self.node_index_map[self.nodes[fpos].vertex]
        self.nodes[fpos], self.nodes[spos] = self.nodes[spos], self.nodes[fpos]

    # The function that min heapifies a node by swapping it with its smallest child
    # Runtime: O(log(n))
    def minHeapify(self, pos):
        if not self.isLeaf(pos):
            if (self.nodes[pos].weight > self.nodes[self.leftChild(pos)].weight or
                self.nodes[pos].weight > self.nodes[self.rightChild(pos)].weight):
                if self.nodes[self.leftChild(pos)].weight < self.nodes[self.rightChild(pos)].weight:
                    self.swap(pos, self.leftChild(pos))
                    self.minHeapify(self.leftChild(pos))
                else:
                    self.swap(pos, self.rightChild(pos))
                    self.minHeapify(self.rightChild(pos))

    # The function that inserts a node into the priority queue
    # Runtime: O(log(n))
    def insert(self, item):
        if self.size >= self.maxsize:
            return
        self.size += 1
        self.nodes[self.size] = item
        self.node_index_map[item.vertex] = self.size
        current = self.size
        while self.nodes[current].weight < self.nodes[self.parent(current)].weight:
            self.swap(current, self.parent(current))
            current = self.parent(current)

    # The function that removes the minimum node from the priority queue
    # Runtime: O(log(n))
    def remove_min(self):
        popped = self.nodes[self.FRONT]
        self.node_index_map.pop(popped.vertex)
        self.nodes[self.FRONT] = self.nodes[self.size]
        self.node_index_map[self.nodes[self.FRONT].vertex] = self.FRONT
        self.size -= 1
        self.minHeapify(self.FRONT)
        return popped

    # The function that makes the priority queue
    # Runtime: O(n)
    def make_queue(self, source_node, verticies):
        for vertex in verticies:
            if vertex != source_node:
                self.insert(QueueNode(float('inf'), vertex))

    # The function that decreases the key of a node in the priority queue
    # Runtime: O(log(n))
    def decrease_key(self, vertex, new_weight):
        index = self.node_index_map[vertex]
        self.nodes[index].weight = new_weight
        while self.nodes[index].weight < self.nodes[self.parent(index)].weight:
            self.swap(index, self.parent(index))
            index = self.parent(index)



class NetworkRoutingSolver:
    def __init__( self):
        pass

    def initializeNetwork( self, network ):
        assert( type(network) == CS312Graph )
        self.network = network

    # The function that returns the shortest path from the source node to the destination node
    # Runtime: O(n)
    def getShortestPath( self, destIndex):
        self.dest = destIndex
        if self.previous[self.dest] == None:
            return {'cost':float('inf'), 'path':[]}
        total_length = self.distance[self.dest]
        path_edges = []
        current_node = self.network.nodes[self.dest]
        while current_node.node_id != self.source:
            if self.previous[current_node.node_id] == None:
                return {'cost':float('inf'), 'path':[]}
            path_edges.append((current_node.loc, self.previous[current_node.node_id].loc, '{:.0f}'.format(self.distance[current_node.node_id] - self.distance[self.previous[current_node.node_id].node_id])))
            current_node = self.previous[current_node.node_id]
        path_edges.reverse()
        return {'cost':total_length, 'path':path_edges}

    # The function that runs Dijkstra's algorithm to determine the shortest paths
    # Runtime: O(n^2) array implementation, O(nlog(n)) heap implementation
    def computeShortestPaths( self, srcIndex, use_heap=False ):
        self.source = srcIndex
        t1 = time.time()

        self.distance = []
        for i in range(len(self.network.nodes)):
            self.distance.append(float('inf'))
        self.distance[self.source] = 0
        self.previous = []
        for i in range(len(self.network.nodes)):
            self.previous.append(None)

        if use_heap:
            queue = HeapQueue(len(self.network.nodes), self.network.nodes[self.source])
            queue.make_queue(self.network.nodes[self.source], self.network.nodes)
        else:
            queue = ArrayQueue()
            queue.make_queue(self.network.nodes[self.source], self.network.nodes)

        while len(queue.nodes) > 0 and queue.size > 0:
            u = queue.remove_min()
            for edge in u.vertex.neighbors:
                alt = self.distance[u.vertex.node_id] + edge.length
                if alt < self.distance[edge.dest.node_id]:
                    self.distance[edge.dest.node_id] = alt
                    self.previous[edge.dest.node_id] = u.vertex
                    queue.decrease_key(edge.dest, alt)

        t2 = time.time()
        return (t2-t1)


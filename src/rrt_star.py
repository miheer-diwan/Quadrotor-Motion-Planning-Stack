from scipy import spatial
import numpy as np
from map import Map, Cuboid

class Node:
    def __init__(self, x, y, z):
        self.x = x       # coordinate
        self.y = y       #coordinate
        self.z = z        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost

    def __eq__(self, other):
        if((self.x-other.x)**2 + (self.y-other.y)**2 + (self.z-other.z)**2 <= 0.0001):
            return True
        else:
            return False

class RRT_star:
    def __init__(self, map):
        self.map = map
        pass

    def dis(self, node1, node2):
        euc_dis = np.sqrt((node2.x-node1.x)**2 + (node2.y-node1.y)**2 + (node2.z-node1.z)**2)
        return euc_dis

    def check_collision(self, node1, node2, res=0.1):
        startPoint = np.array([node1.x, node1.y, node1.z])
        endPoint = np.array([node2.x, node2.y, node2.z])

        v = endPoint - startPoint
        length = np.linalg.norm(v)

        num_sampled_points = int(np.floor(length/res))

        for i in range(num_sampled_points):
            sampled_point = startPoint + i*res*v/length
            if self.map.isPointInCollision(sampled_point):
                return True
            
        return False

    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        prob = np.random.random()
        if prob > goal_bias:
            point = Node(np.random.uniform(self.map.boundary.xmin,self.map.boundary.xmax),\
                            np.random.uniform(self.map.boundary.ymin,self.map.boundary.ymax),\
                            np.random.uniform(self.map.boundary.zmin,self.map.boundary.zmax))
        else:
            point = Node(self.goal.x, self.goal.y,self.goal.z)
        # print("point:(%f,%f,%f)" % (point.x, point.y, point.z))
        # print(np.random.randint(0,self.size_col,1)[0])
        return point

    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        kdtree = spatial.KDTree([(v.x,v.y,v.z) for v in self.vertices])

        dist,ind = kdtree.query((point.x,point.y,point.z),1) 

        return self.vertices[ind]

    def get_neighbors(self, new_node, neighbor_radius):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_radius - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        kdtree = spatial.KDTree([(v.x,v.y,v.z) for v in self.vertices])

        n = kdtree.query_ball_point((new_node.x,new_node.y,new_node.z),neighbor_radius,workers=4)
        neighbors=[]

        for i in n:
            if(not self.check_collision(self.vertices[i],new_node)):
                neighbors.append(i)

        return neighbors

    def rewire(self, new_node, neighbors,nearest_node):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        # print(neighbors)
        for neighbor in neighbors:
            # print(neighbor)
            neighbor_node = self.vertices[neighbor]
            cost = self.dis(neighbor_node, new_node) + new_node.cost
            if neighbor_node != nearest_node:
                if not self.check_collision(neighbor_node, new_node) and cost < neighbor_node.cost:
                    neighbor_node.cost = cost
                    neighbor_node.parent = new_node
                    # Check if the rewiring caused a shorter path to the start node
                    # self.update_cost_to_start(neighbor_node)

    def sample_new_point(self, neighbor_radius=5):
        point = self.get_new_point(0.1)
        if(self.map.isPointInMapBoundary([point.x,point.y, point.z])):

            nearest_node = self.get_nearest_node(point)

            distance = self.dis(point,nearest_node)

            if(distance>10):
                point.x = nearest_node.x + (((point.x - nearest_node.x)/distance)*10)
                point.y = nearest_node.y + (((point.y - nearest_node.y)/distance)*10)
                point.z = nearest_node.z + (((point.z - nearest_node.z)/distance)*10)


            neighbors = self.get_neighbors(point,neighbor_radius)
            cost = nearest_node.cost + self.dis(point,nearest_node)
            idx = 0
            for i in neighbors:
                if(self.vertices[i].cost + self.dis(point,self.vertices[i])<cost):
                    cost = self.vertices[i].cost + self.dis(point,self.vertices[i])
                    nearest_node = self.vertices[i]

            if (self.check_collision(point,nearest_node)==False):
                point.parent =  nearest_node  
                point.cost = point.parent.cost + self.dis(point,nearest_node)
                self.rewire(point,neighbors,nearest_node)
                self.vertices.append(point)  

                if (point == self.goal):
                    # print("Point: (%f, %f, %f) - Goal: (%f, %f, %f)" % (point.x, point.y, point.z, self.goal.x,self.goal.y,self.goal.z))
                    self.found = True
                    self.path_end = point
            
            return self.found
    def reset_search_reslut(self):
        self.vertices = []
        self.found = False
        self.num_samples = 0

    def set_start_and_goal(self, start, goal):
        self.start = Node(start[0], start[1], start[2])
        self.vertices.append(self.start)
        self.goal = Node(goal[0], goal[1], goal[2])
    
    def set_max_samples(self, max_samples = 1000):
        self.max_samples = max_samples

    def search_step(self, neighbor_radius=5, step_size = 1):
        for i in range(step_size):
            self.found = self.sample_new_point(neighbor_radius)
            self.num_samples += 1
            if(self.num_samples >= self.max_samples):
                break
        return self.found
    
    def search(self, neighbor_radius=5):   
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        while self.num_samples < self.max_samples:
            # print(i)
            # print(len(self.vertices))
            self.found = self.sample_new_point(neighbor_radius)
            self.num_samples += 1

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.path_end.cost
            # print("Point: (%d, %d, %d) - Goal: (%d, %d, %d)" % (point.x, point.y, point.z, self.goal.x,self.goal.y,self.goal.z))
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")
            
    def remove_duplicates(self, vertices):
        newVertices =[]
        for vertex in vertices:
            if len(newVertices):
                if vertex != newVertices[-1]:
                    newVertices.append(vertex)
            else:
                newVertices.append(vertex)
        return newVertices
    
    def retrieve_path(self):
        current = self.path_end
        vertices = []
        path_start_vertices = []
        path_end_vertices = []
        while(current.parent is not None):
            vertices.append([current.x, current.y, current.z])
            path_start_vertices.append([current.parent.x, current.parent.y, current.parent.z])
            path_end_vertices.append([current.x, current.y, current.z])
            current = current.parent
        vertices.append([current.x, current.y, current.z])
        
        vertices.reverse()
        vertices = self.remove_duplicates(vertices)
        # path_start_vertices = self.remove_duplicates(path_start_vertices)
        # path_end_vertices = self.remove_duplicates(path_end_vertices)
        return vertices, path_start_vertices, path_end_vertices

    def get_vertices(self):
        vertices = []
        for vertex in self.vertices:
            vertices.append([vertex.x, vertex.y, vertex.z])

        return vertices
    
    def get_edges(self):
        start_vertices = []
        end_vertices = []

        for vertex in self.vertices:
            parent = vertex.parent
            if parent:
                start_vertices.append([parent.x, parent.y, parent.z])
                end_vertices.append([vertex.x, vertex.y, vertex.z])
        return start_vertices, end_vertices


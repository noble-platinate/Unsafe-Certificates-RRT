"""
add resolution, threshold to collision check
check intermediate points for collision
add dynamic map forming
visualization gaps
"""
# x is width, y is height
# just do x,y everywhere

from utils import euclidean_distance
import cv2
import numpy as np
import random
import math

class Node:
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

class Edge:
    def __init__(self,node_1,node_2,cost):
        self.node_1 = node_1
        self.node_2 = node_2
        self.cost = cost

class Map:
    def __init__(self, height, width, step_size, start, goal):
        self.height = height
        self.width = width

        self.step_size = step_size
        self.map =  np.ones((self.height,self.width,3), np.uint8) * 255

        self.start = Node(start[0],start[1])
        self.goal = Node(goal[0],goal[1])

        self.obstacle_list = []
        self.nodes = [self.start]
        self.edges = []
        self.x_soln = []

        self.solution_found = False
        self.c_best = np.inf
        self.bot_safety_distance = 1.0
        self.first_sample = True

    def add_obstacle(self,x,y,width,height):
        for i in range(x,x+width):
            for j in range(y,y+height):
                self.map[j,i,0] = 0
                self.map[j,i,1] = 0
                self.map[j,i,2] = 0
                self.obstacle_list.append((j,i))
    
    def display_map(self,x_rand,best_path_found = False):
        # img = self.map
        img = np.ones((self.height,self.width,3), np.uint8) * 255 # numpy array
        img = cv2.circle(img,(self.start.x,self.start.y),5,(0,255,0),-1)
        img = cv2.circle(img,(self.goal.x,self.goal.y),5,(0,0,255),-1)
        # img = cv2.circle(img,(x_rand.x,x_rand.y),5,(0,0,0),-1)

        for edge in self.edges:
            node_1 = (edge.node_1.x,edge.node_1.y)
            node_2 = (edge.node_2.x,edge.node_2.y)
            img = cv2.line(img,node_1,node_2,(255,0,0),1)

        if(best_path_found):
            curr_node = self.goal
            while(curr_node is not None):
                self.x_soln.append(curr_node)
                curr_node = curr_node.parent
        
            for i in range(len(self.x_soln)-1):
                node_1 = (self.x_soln[i].x,self.x_soln[i].y)
                node_2 = (self.x_soln[i+1].x,self.x_soln[i+1].y)
                img = cv2.line(img,node_1,node_2,(0,0,255),4)

        for obstacle in self.obstacle_list:
            img[obstacle[0],obstacle[1]] = (0,0,0)

        for node in self.nodes:
            curr_node = node
            path = []
            while(curr_node is not None):
                path.append(curr_node)
                curr_node = curr_node.parent
        
            for i in range(len(path)-1):
                node_1 = (path[i].x,path[i].y)
                node_2 = (path[i+1].x,path[i+1].y)
                img = cv2.line(img,node_1,node_2,(255,0,0),1)

        cv2.imshow("img",img)
        cv2.waitKey(10)
    
    def euclidean_distance(self, node_1, node_2):
        return np.sqrt( (node_1.x - node_2.x)**2 + (node_1.y - node_2.y)**2)
    
    def sample(self, x_start, x_goal, c_max):
        # _1 = x_start
        # _2 = x_goal
        # _3 = c_max

        random_x = random.randint(0,self.width)
        random_y = random.randint(0,self.height)

        random_node = Node(random_x, random_y)

        return random_node
    
    def nearest_node(self, x_rand):
        cost = np.inf
        nearest_node_to_sample = None
        found_nearest_node = False

        for node_ in self.nodes:
            if(self.euclidean_distance(x_rand,node_) < cost):
                cost = self.euclidean_distance(x_rand,node_)
                nearest_node_to_sample = node_
                found_nearest_node = True

        if(nearest_node_to_sample is not None):
            return found_nearest_node, nearest_node_to_sample, cost
        else:
            return found_nearest_node, x_rand, 0
    
    def steer(self, x_nearest, x_rand, cost):
        det = self.euclidean_distance(x_nearest, x_rand)

        if(det <= self.step_size):
            return x_rand, cost
        else:
            step_x = int(x_nearest.x + (x_rand.x - x_nearest.x) * self.step_size / det)
            step_y = int(x_nearest.y + (x_rand.y - x_nearest.y) * self.step_size / det)
            
            stepped_node = Node(step_x, step_y)

            cost = self.euclidean_distance(stepped_node, x_nearest)
            
            return stepped_node, cost
    
    def set_node_cost(self,node):
        node_cost = 0.0
        curr_node = node
        if(curr_node.parent is None):
            node_cost = 0.0
        else:
            node_cost = node.parent.cost + self.euclidean_distance(node,node.parent)
        return node_cost
    
    def collision_free(self, x_nearest, x_new):
        if(self.map[x_nearest.y,x_nearest.x,0] == 0 and self.map[x_nearest.y,x_nearest.x,1] == 0 and self.map[x_nearest.y,x_nearest.x,2] == 0):
            return 0
        if(self.map[x_new.y,x_new.x,0] == 0 and self.map[x_new.y,x_new.x,1] == 0 and self.map[x_new.y,x_new.x,2] == 0):
            return 0
        return 1

    def print_nodes(self):
        for node in self.nodes:
            print(node.x,node.y)
    
    def print_node_list(self,nodes_in_range):
        for node in nodes_in_range:
            print(node.x,node.y)

    def print_edges(self):
        for edge in self.edges:
            print(f"Edge between ({edge.node_1.x},{edge.node_1.y}) and ({edge.node_2.x},{edge.node_2.y}), cost : {edge.cost}")
    
    def print_nodes_and_edges(self):
        print("Nodes : ",self.print_nodes())
        print("Edges : ",self.print_edges())

    def print_best_path(self):
        for node in self.x_soln:
            print(node.x,node.y)
    
    def print_parent(self,node):
        print(f"Parent : ({node.parent.x},{node.parent.y})")
    
    def get_nodes_in_radius(self,radius,x_new):
        nodes_in_radius = []
        for node in self.nodes:
            if(self.euclidean_distance(node,x_new) <= radius):
                nodes_in_radius.append(node)
        return nodes_in_radius

    def delete_edge(self,node_1,node_2):
        for edge in self.edges:
            if((edge.node_1 == node_1 and edge.node_2 == node_2) or (edge.node_1 == node_2 and edge.node_2 == node_1)):
                self.edges.remove(edge)
    
    def delete_all_edges(self,node):
        edges_to_remove = []
        for edge in self.edges:
            if(edge.node_1 == node or edge.node_2 == node):
                edges_to_remove.append(edge)
        # to avoid memory issues
        for edge_ in edges_to_remove:
            self.edges.remove(edge_)
    
    def revise_costs(self):
        for node in self.nodes:
            node.cost = self.set_node_cost(node)

    def rewire(self,x_new,nodes_in_radius):
        
        for node in self.nodes:
            node.cost = self.set_node_cost(node)

        cost = np.inf
        best_cost_node = None
        first_sample = False

        # connect to best-case node
        for node in nodes_in_radius:
            if(node.cost + self.euclidean_distance(node,x_new) < cost):
                best_cost_node = node
                cost = node.cost + self.euclidean_distance(node,x_new)
        
        self.delete_all_edges(x_new)
        edge = Edge(best_cost_node,x_new,self.euclidean_distance(best_cost_node,x_new))

        x_new.parent = best_cost_node
        x_new.cost = self.euclidean_distance(best_cost_node,x_new) + best_cost_node.cost
        edge.node_1 = x_new
        edge.node_2 = x_new.parent
        edge.cost = self.euclidean_distance(x_new,x_new.parent)
        self.edges.append(edge)
        if(len(self.nodes) == 1):
            print("1 node but still rewiring!")

        nodes_in_radius.remove(best_cost_node)

        # form new edge
        self.nodes.append(x_new)

        parent_edge = Edge(x_new, x_new.parent, self.euclidean_distance(x_new,x_new.parent))

        for node in nodes_in_radius:
            if(self.collision_free(x_new,node)):
                if(x_new.cost + self.euclidean_distance(x_new,node) < node.cost):
                    node.cost = x_new.cost + self.euclidean_distance(x_new,node)
                    node.parent = x_new
                    self.delete_all_edges(node)
                    edge.node_1 = x_new
                    edge.node_2 = node
                    edge.cost = self.euclidean_distance(x_new,node)
                    self.edges.append(edge)
                else:
                    pass
            parent_edge.node_1 = node
            parent_edge.node_2 = node.parent
            parent_edge.cost = self.euclidean_distance(node,node.parent)
            self.edges.append(parent_edge)
        best_cost_edge = Edge(x_new,best_cost_node,self.euclidean_distance(x_new,best_cost_node))
        self.edges.append(best_cost_edge)

def debug_rewiring():
    start = [10,10]
    goal = [90,90]
    check_radius = 10.0

    map = Map(100,100,8,start,goal)

    node_1 = Node(20,20)
    node_1.parent = map.start
    node_1.cost = map.set_node_cost(node_1)
    map.nodes.append(node_1)

    node_2 = Node(30,20)
    node_2.parent = node_1
    node_2.cost = map.set_node_cost(node_2)
    map.nodes.append(node_2)

    node_3 = Node(26,24)
    node_3.parent = node_2
    node_3.cost = map.set_node_cost(node_3)
    map.nodes.append(node_3)

    edge_1 = Edge(map.start,node_1,map.euclidean_distance(map.start,node_1))
    map.edges.append(edge_1)

    edge_2 = Edge(node_1,node_2,map.euclidean_distance(node_2,node_1))
    map.edges.append(edge_2)

    edge_3 = Edge(node_2,node_3,map.euclidean_distance(node_2,node_3))
    map.edges.append(edge_3)

    x_rand = Node(23,22)
    x_rand.parent = map.start
    x_rand.cost = map.euclidean_distance(map.start,x_rand)

    nodes_in_range = map.get_nodes_in_radius(check_radius,x_rand)

    map.rewire(x_rand,nodes_in_range)

if __name__=="__main__":
    debug_rewiring()
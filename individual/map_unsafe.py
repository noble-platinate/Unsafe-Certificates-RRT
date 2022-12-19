# x is width, y is height
# just do x,y everywhere

import cv2
import numpy as np
import random
import math
import time

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

class Safety_Certificate:
    def __init__(self, node, half_distance):
        self.node=node
        self.half_distance=half_distance
        
class Map:
    def __init__(self, height, width, step_size, start, goal):
        self.height = height
        self.width = width

        self.step_size = step_size

        self.start = Node(start[0],start[1])
        self.goal = Node(goal[0],goal[1])
        self.map = np.ones((self.height, self.width, 3), np.uint8) * 255
        self.map_padding = np.ones((self.height, self.width, 3), np.uint8) * 255

        self.obstacle_list = []
        self.obstacle_list_padding=[]
        self.freenodes = [self.start]
        self.obstaclenodes = []
        self.edges = []
        self.x_soln = []

        self.solution_found = False
        self.bot_safety_distance = 1.0
        self.first_sample = True

        self.major_axis = 0.0
        self.minor_axis = 0.0

        self.best_cost_for_informed = np.inf
        self.show_edges = 0
        self.show_sample = 0
        self.show_ellipse = 0
        self.safety_certificates = []
        self.unsafety_certificates = []
        self.unsafety_cert_check_times=0
        self.unsafe_collision_check_times=0

        self.normal_collision_check_times=0
        self.safety_cert_check_times=0
        
    def add_obstacle(self,x,y,width,height,clearance):
        flag = 1
        for i in range(x - clearance , x + width + clearance):
            for j in range(y - clearance , y + height + clearance):
                if(i == self.start.x and j == self.start.y):
                    flag = 0
                    break
                if(i == self.goal.x and j == self.goal.y):
                    flag = 0
                    break
        
        if(flag):
            for i in range(x,x+width):
                for j in range(y,y+height):
                    self.map[i,j,0] = 0
                    self.map[i,j,1] = 0
                    self.map[i,j,2] = 0
                    self.obstacle_list.append((j,i))

            for i in range(x-2, x+width+2):
                for j in range(y-2, y+height+2):
                    self.map_padding[i, j, 0] = 0
                    self.map_padding[i, j, 1] = 0
                    self.map_padding[i, j, 2] = 0
                    self.obstacle_list_padding.append((j, i))
    
    def display_map(self,x_rand,best_path_found = False):
        best_cost = 0.0

        img = np.ones((self.height, self.width, 3), np.uint8) * 255

        img = cv2.circle(
            img, (self.start.x, self.start.y), 5, (0, 255, 0), -1)
        img = cv2.circle(
            img, (self.goal.x, self.goal.y), 5, (0, 0, 255), -1)

        if (best_path_found):
            curr_node = self.goal
            while (curr_node is not None):
                self.x_soln.append(curr_node)
                curr_node = curr_node.parent

            for i in range(len(self.x_soln)-1):
                node_1 = (self.x_soln[i].x, self.x_soln[i].y)
                node_2 = (self.x_soln[i+1].x, self.x_soln[i+1].y)
                img = cv2.line(img, node_1, node_2, (0, 0, 255), 4)

        for node in self.freenodes:
            curr_node = node
            path = []
            while(curr_node is not None):
                path.append(curr_node)
                curr_node = curr_node.parent
        
            for i in range(len(path)-1):
                node_1 = (path[i].x,path[i].y)
                node_2 = (path[i+1].x,path[i+1].y)
                if(self.show_edges):
                    img = cv2.line(img,node_1,node_2,(255,0,0),1)
                node_1_N = Node(node_1[0],node_1[1])
                node_2_N = Node(node_2[0],node_2[1])
                best_cost += self.euclidean_distance(node_1_N,node_2_N)

        for obstacle in self.obstacle_list:
            img[obstacle[0], obstacle[1]] = (0, 0, 0)

        map1 = img.copy()

        for safety_certi in self.safety_certificates:
            map2 = map1.copy()
            node_safe = safety_certi.node
            half_distance = safety_certi.half_distance
            cv2.circle(map2, (node_safe.x, node_safe.y),
                       half_distance, (100, 0, 10), -1)
            map1=cv2.addWeighted(
                map2, 0.4, map1, 1 - 0.4, 0)

        map2 = map1.copy()

        for unsafety_certi in self.unsafety_certificates:
            map3 = map2.copy()
            node_unsafe = unsafety_certi.node
            half_distance = unsafety_certi.half_distance
            cv2.circle(map3, (node_unsafe.x, node_unsafe.y),
                       half_distance, (0, 100, 10), -1)
            map2 = cv2.addWeighted(
                map3, 0.8, map2, 0.2, 0)

        cv2.imshow("self.map", map2)
        cv2.waitKey(1)

        return best_cost

    def euclidean_distance(self, node_1, node_2):
        try:
            return np.sqrt( (node_1.x - node_2.x)**2 + (node_1.y - node_2.y)**2)
        except:
            return 0
    
    def sample(self):
        random_x = random.randint(0,self.width)
        random_y = random.randint(0,self.height)

        random_node = Node(random_x, random_y)

        return random_node
    
    def nearest_node(self, x_rand):
        cost = np.inf
        nearest_node_to_sample = None
        found_nearest_node = False

        for node_ in self.freenodes:
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

    def safety_cert(self,node):
        for half_distance in range(1000):
            try:
                for x in range(half_distance): 
                    if (self.safety_certified(Node(node.x+x, node.y+half_distance))==None):
                        if (self.normal_collision_check(Node(node.x+x, node.y+half_distance))):
                            return max(0,half_distance-1)
                    if (self.safety_certified(Node(node.x-x, node.y+half_distance)) == None):
                        if (self.normal_collision_check(Node(node.x-x, node.y+half_distance))):
                            return max(0, half_distance-1)
                    if (self.safety_certified(Node(node.x+x, node.y-half_distance)) == None):
                        if (self.normal_collision_check(Node(node.x+x, node.y-half_distance))):
                            return max(0, half_distance-1)
                    if (self.safety_certified(Node(node.x-x, node.y-half_distance)) == None):
                        if (self.normal_collision_check(Node(node.x-x, node.y-half_distance))):
                            return max(0, half_distance-1)
                for y in range(half_distance):
                    if (self.safety_certified(Node(node.x+half_distance, node.y+y)) == None):
                        if (self.normal_collision_check(Node(node.x+half_distance, node.y+y))):
                            return max(0, half_distance-1)
                    if (self.safety_certified(Node(node.x+half_distance, node.y-y)) == None):
                        if (self.normal_collision_check(Node(node.x+half_distance, node.y-y))):
                            return max(0, half_distance-1)
                    if (self.safety_certified(Node(node.x-half_distance, node.y+y)) == None):
                        if (self.normal_collision_check(Node(node.x-half_distance, node.y+y))):
                            return max(0, half_distance-1)
                    if (self.safety_certified(Node(node.x-half_distance, node.y-y)) == None):
                        if (self.normal_collision_check(Node(node.x-half_distance, node.y-y))):
                            return max(0, half_distance-1)
            except:
                return max(0, half_distance-1)
        return max(0, half_distance-1)

    def unsafety_cert(self, node):
        for half_distance in range(1000):
            try:
                for x in range(half_distance):
                    if (self.unsafety_certified(Node(node.x+x, node.y+half_distance)) == None):
                        if (self.unsafe_collision_check(Node(node.x+x, node.y+half_distance))):
                            return max(0, half_distance-1)
                    if (self.unsafety_certified(Node(node.x-x, node.y+half_distance)) == None):
                        if (self.unsafe_collision_check(Node(node.x-x, node.y+half_distance))):
                            return max(0, half_distance-1)
                    if (self.unsafety_certified(Node(node.x+x, node.y-half_distance)) == None):
                        if (self.unsafe_collision_check(Node(node.x+x, node.y-half_distance))):
                            return max(0, half_distance-1)
                    if (self.unsafety_certified(Node(node.x-x, node.y-half_distance)) == None):
                        if (self.unsafe_collision_check(Node(node.x-x, node.y-half_distance))):
                            return max(0, half_distance-1)
                for y in range(half_distance):
                    if (self.unsafety_certified(Node(node.x+half_distance, node.y+y)) == None):
                        if (self.unsafe_collision_check(Node(node.x+half_distance, node.y+y))):
                            return max(0, half_distance-1)
                    if (self.unsafety_certified(Node(node.x+half_distance, node.y-y)) == None):
                        if (self.unsafe_collision_check(Node(node.x+half_distance, node.y-y))):
                            return max(0, half_distance-1)
                    if (self.unsafety_certified(Node(node.x-half_distance, node.y+y)) == None):
                        if (self.unsafe_collision_check(Node(node.x-half_distance, node.y+y))):
                            return max(0, half_distance-1)
                    if (self.unsafety_certified(Node(node.x-half_distance, node.y-y)) == None):
                        if (self.unsafe_collision_check(Node(node.x-half_distance, node.y-y))):
                            return max(0, half_distance-1)
            except:
                return max(0, half_distance-1)
        return max(0, half_distance-1)

    def safety_certified(self,node):
        self.safety_cert_check_times = self.safety_cert_check_times+1
        for certificate in self.safety_certificates:
            half_distance = certificate.half_distance
            if (abs(node.x-certificate.node.x) < half_distance and abs(node.y-certificate.node.y) < half_distance):
                return certificate.node
        return None
    
    def unsafety_certified(self, node):
        self.unsafety_cert_check_times = self.unsafety_cert_check_times+1
        for certificate in self.unsafety_certificates:
            half_distance = certificate.half_distance
            if (abs(node.x-certificate.node.x) < half_distance and abs(node.y-certificate.node.y) < half_distance):
                return certificate.node
        return None

    def normal_collision_check(self,node):
        self.normal_collision_check_times = self.normal_collision_check_times+1
        if (self.map_padding[node.x, node.y, 0] == 0 and self.map_padding[node.x, node.y, 1] == 0 and self.map_padding[node.x, node.y, 2] == 0):
            return 1
        return 0
    
    def unsafe_collision_check(self, node):
        self.unsafe_collision_check_times = self.unsafe_collision_check_times+1
        if (self.map_padding[node.x, node.y, 0] == 255 and self.map_padding[node.x, node.y, 1] == 255 and self.map_padding[node.x, node.y, 2] == 255):
            return 1
        return 0

    def collision_free(self, x_nearest, x_new, resolution = 0.1):
        node_safe = self.safety_certified(x_new)
        node_unsafe = self.unsafety_certified(x_new)

        if(node_unsafe!=None):
            self.obstaclenodes.append(x_new)
            return 0

        if(node_safe!=None):
            x_nearest_safe=self.safety_certified(x_nearest)
            if(node_safe==x_nearest or x_nearest_safe==node_safe):
                return 1    
            else:
                parts = int(1/resolution)
                for i in range(1, parts):
                    section_x = int((x_nearest.x*i + x_new.x*(parts-i))/parts)
                    section_y = int((x_nearest.y*i + x_new.y*(parts-i))/parts)
                    node_safe_=self.safety_certified(Node(section_x, section_y))
                    if(node_safe_!=None):
                        if (node_safe_ == x_nearest or node_safe_ == x_nearest_safe):
                            return 1
                    elif(self.normal_collision_check(Node(section_x, section_y))):
                        return 0
        
        elif(self.normal_collision_check(x_new)):
            radius = self.unsafety_cert(x_new)
            self.unsafety_certificates.append(Safety_Certificate(x_new, radius))
            self.obstaclenodes.append(x_new)
            return 0
        else:
            x_nearest_safe=self.safety_certified(x_nearest)
            radius = self.safety_cert(x_new)            
            self.safety_certificates.append(Safety_Certificate(x_new, radius))

            parts = int(1/resolution)
            for i in range(1, parts):
                section_x = int((x_nearest.x*i + x_new.x*(parts-i))/parts)
                section_y = int((x_nearest.y*i + x_new.y*(parts-i))/parts)
                node_safe_ = self.safety_certified(
                    Node(section_x, section_y))
                if (node_safe_ != None):
                    if (node_safe_ == x_nearest or node_safe_ == x_nearest_safe):
                        return 1
                elif (self.normal_collision_check(Node(section_x, section_y))):
                    return 0

        return 1
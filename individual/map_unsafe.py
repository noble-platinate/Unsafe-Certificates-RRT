# x is width, y is height
# just do x,y everywhere

import cv2
import numpy as np
import random
import math
import time
from sklearn.neighbors import KDTree


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0


class Edge:
    def __init__(self, node_1, node_2, cost):
        self.node_1 = node_1
        self.node_2 = node_2
        self.cost = cost


class Safety_Certificate:
    def __init__(self, node, half_distance):
        self.node = node
        self.half_distance = half_distance


class Map:
    def __init__(self, height, width, step_size, start, goal):
        self.height = height
        self.width = width

        self.step_size = step_size

        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])

        self.obstacle_list = []
        self.obstacle_list_padding = []
        self.obstacle_tree = None
        self.nonobstacle_list = []
        self.nonobstacle_tree = None
        self.freenodes = [start]
        self.nodes = [self.start]
        self.obstaclenodes = []
        self.edges = []
        self.x_soln = []

        self.solution_found = False
        self.bot_safety_distance = 1.0
        self.first_sample = True

        self.major_axis = 0.0
        self.minor_axis = 0.0

        self.show_edges = 1
        self.show_sample = 0
        self.show_ellipse = 0

        self.safety_certificates_centers = []
        self.safety_certificates_radius = {}
        self.unsafety_certificates_centers = []
        self.unsafety_certificates_radius = {}
        self.certifier_of = {}
        self.uncertifier_of = {}

    def add_obstacle(self, x, y, width, height, clearance):
        flag = 1
        for i in range(x - clearance, x + width + clearance):
            for j in range(y - clearance, y + height + clearance):
                if (i == self.start.x and j == self.start.y):
                    flag = 0
                    break
                if (i == self.goal.x and j == self.goal.y):
                    flag = 0
                    break

        if (flag):
            for i in range(x, x+width):
                for j in range(y, y+height):
                    self.obstacle_list.append([j, i])

            for i in range(x-2, x+width+2):
                for j in range(y-2, y+height+2):
                    self.obstacle_list_padding.append([j, i])

    def display_map(self):

        img = np.ones((self.height, self.width, 3), np.uint8) * 255

        img = cv2.circle(
            img, (self.start.x, self.start.y), 5, (0, 255, 0), -1)
        img = cv2.circle(
            img, (self.goal.x, self.goal.y), 5, (0, 0, 255), -1)

        for node in self.nodes:
            path = []
            while (node is not None):
                path.append(node)
                node = node.parent
            for i in range(len(path)-1):
                node_1 = (path[i].x, path[i].y)
                node_2 = (path[i+1].x, path[i+1].y)
                if (self.show_edges):
                    img = cv2.line(img, node_1, node_2, (255, 0, 0), 2)

        for obstacle in self.obstacle_list:
            img[obstacle[0], obstacle[1]] = (0, 0, 0)

        map1 = img.copy()

        for safety_certi in self.safety_certificates_centers:
            map2 = map1.copy()
            x, y = safety_certi
            radius = self.safety_certificates_radius[(x, y)]
            cv2.circle(map2, (x, y),
                       int(radius), (100, 0, 10), -1)
            map1 = cv2.addWeighted(
                map2, 0.4, map1, 1 - 0.4, 0)

        for safety_certi in self.unsafety_certificates_centers:
            map2 = map1.copy()
            x, y = safety_certi
            radius = self.unsafety_certificates_radius[(x, y)]
            cv2.circle(map2, (x, y),
                       int(radius), (0, 200, 10), -1)
            map1 = cv2.addWeighted(
                map2, 0.4, map1, 1 - 0.4, 0)

        cv2.imshow("self.map", map1)
        cv2.waitKey(1)

        return 1

    def euclidean_distance(self, node_1, node_2):
        try:
            return np.sqrt((node_1.x - node_2.x)**2 + (node_1.y - node_2.y)**2)
        except:
            return 0

    def sample(self):
        if np.random.random() > 0:
            random_x = int(np.random.uniform(0, self.width))
            random_y = int(np.random.uniform(0, self.height))
            random_node = Node(random_x, random_y)

            return random_node
        else:
            return self.goal

    def nearest_node(self, x_rand):
        nearest_node_to_sample = None
        found_nearest_node = False

        tree = KDTree(self.freenodes)
        nearest_node_to_sample, node_near_idx = tree.query(
            [[x_rand.x, x_rand.y]], k=1)
        node_near = Node(self.freenodes[node_near_idx[0][0]][0],
                         self.freenodes[node_near_idx[0][0]][1])

        found_nearest_node = True
        return found_nearest_node, node_near, nearest_node_to_sample[0][0]

    def steer(self, x_nearest, x_rand, cost):
        det = self.euclidean_distance(x_nearest, x_rand)

        if (det <= self.step_size):
            return x_rand, cost
        else:
            step_x = int(x_nearest.x + (x_rand.x - x_nearest.x)
                         * self.step_size / det)
            step_y = int(x_nearest.y + (x_rand.y - x_nearest.y)
                         * self.step_size / det)

            stepped_node = Node(step_x, step_y)

            cost = self.euclidean_distance(stepped_node, x_nearest)

            return stepped_node, cost

    def set_node_cost(self, node):
        node_cost = 0.0
        curr_node = node
        if (curr_node.parent is None):
            node_cost = 0.0
        else:
            node_cost = node.parent.cost + \
                self.euclidean_distance(node, node.parent)
        return node_cost

    def safety_certified(self, node):
        if (len(self.safety_certificates_centers) > 0):
            k = min(len(self.safety_certificates_centers), 5)
            tree = KDTree(self.safety_certificates_centers)
            dist, node_near_idx = tree.query([[node.x, node.y]], k)

            for idx in range(k):
                node_near = Node(self.safety_certificates_centers[node_near_idx[0][idx]][0],
                                 self.safety_certificates_centers[node_near_idx[0][idx]][1])

                rad = self.safety_certificates_radius[(
                    node_near.x, node_near.y)]
                if (dist[0][idx] <= rad):
                    self.certifier_of[(node.x, node.y)] = (
                        node_near.x, node_near.y)
                    return node_near

        return None
    
    def unsafety_certified(self, node):
        if (len(self.unsafety_certificates_centers) > 0):
            k = min(len(self.unsafety_certificates_centers), 10)
            tree = KDTree(self.unsafety_certificates_centers)
            dist, node_near_idx = tree.query([[node.x, node.y]], k)

            for idx in range(k):
                node_near = Node(self.unsafety_certificates_centers[node_near_idx[0][idx]][0],
                                 self.unsafety_certificates_centers[node_near_idx[0][idx]][1])

                rad = self.unsafety_certificates_radius[(
                    node_near.x, node_near.y)]
                if (dist[0][idx] <= rad):
                    self.uncertifier_of[(node.x, node.y)] = (
                        node_near.x, node_near.y)
                    return node_near
        return None

    def normal_collision_check(self, node):
        dist, node_near_idx = self.obstacle_tree.query(
            [[node.y, node.x]], k=1)

        if (dist[0][0] > 0):
            self.safety_certificates_centers.append((node.x, node.y))
            self.safety_certificates_radius[(node.x, node.y)] = dist[0][0]
            self.certifier_of[(node.x, node.y)] = (node.x, node.y)
            return False
        else:
            dist, node_near_idx = self.nonobstacle_tree.query([[node.y, node.x]], k=1)

            self.unsafety_certificates_centers.append((node.x, node.y))
            self.unsafety_certificates_radius[(node.x, node.y)] = dist[0][0]
            self.uncertifier_of[(node.x, node.y)] = (node.x, node.y)
            return True

    def collision_free(self, x):

        if (self.safety_certified(x) == None):
            if(self.unsafety_certified(x) != None):
                return 0
            if (self.normal_collision_check(x) == True):
                return 0
            else:
                return 1
        else:
            return 1

    def path_free(self, x_nearest, x_new, resolution=0.1):
        safety_node = self.certifier_of[(x_nearest.x, x_nearest.y)]

        if (self.euclidean_distance(x_nearest, x_new) <= self.safety_certificates_radius[safety_node]):
            return 1
        else:
            parts = int(1/resolution)
            for i in range(1, parts):
                section_x = int((x_nearest.x*i + x_new.x*(parts-i))/parts)
                section_y = int((x_nearest.y*i + x_new.y*(parts-i))/parts)
                node_temp = Node(section_x, section_y)
                dist, node_near_idx = self.obstacle_tree.query(
                    [[node_temp.y, node_temp.x]], k=1)

                if (dist[0][0] == 0):
                    try:
                        return prev_node
                    except:
                        return 0

                prev_node = node_temp
        return 1

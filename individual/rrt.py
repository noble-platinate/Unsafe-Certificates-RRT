import numpy as np
import cv2
import map_normal
import map_safe
import map_unsafe
import map_safe_2
import random
import time
import json
import matplotlib.pyplot as plt
from sklearn.neighbors import KDTree

def add_obstacle(x,y,width,height,clearance,start,goal):
    flag = 1
    for i in range(x - clearance , x + width + clearance):
        for j in range(y - clearance , y + height + clearance):
            if(i == start[0] and j == start[1]):
                flag = 0
                break
            if(i == goal[0] and j == goal[1]):
                flag = 0
                break
    
    if(flag):
        for i in range(x,x+width):
            for j in range(y,y+height):
                map.obstacle_list.append((j, i))
                map1.obstacle_list.append((j, i))
                map2.obstacle_list.append((j, i))
                map3.obstacle_list.append((j, i))

        for i in range(x-2, x+width+2):
            for j in range(y-2, y+height+2):
                map.obstacle_list_padding.append((j, i))
                map1.obstacle_list_padding.append((j, i))
                map2.obstacle_list_padding.append((j, i))
                map3.obstacle_list_padding.append((j, i))


if __name__ == "__main__":

    s_time = time.time()

    f = open('/Users/akshatanshnayak/Desktop/Unsafe-Certificates-RRT/individual/config.json',)
    
    data = json.load(f)
    
    for param in data['parameters']:
        height = param['height']
        width = param['width']
        start_pose = param['start_pose']
        goal_pose = param['goal_pose']
        step_size = param['step_size']
        search_radius = param['search_radius']
        ITERATIONS = param['ITERATIONS']
        show_edges = param['show_edges']
        show_sample = param['show_sample']
        show_ellipse = param['show_ellipse']
        threshold_cost = param['threshold_cost']
        generate_random_map = param['generate_random_map']
        num_random_obstacles = param['num_random_obstacles']
        rand_obstacle_size = param['rand_obstacle_size']
        clearance = param['clearance']
    
    f.close()

    if(search_radius < step_size):
        print("search radius should be > step_size")

    map = map_normal.Map(height, width, step_size, start_pose, goal_pose)
    map1 = map_safe.Map(height, width, step_size, start_pose, goal_pose)
    map2 = map_unsafe.Map(height, width, step_size, start_pose, goal_pose)
    map3 = map_safe_2.Map(height, width, step_size, start_pose, goal_pose)


    map.set_node_cost(map.start)
    map1.set_node_cost(map1.start)
    map2.set_node_cost(map2.start)
    map3.set_node_cost(map3.start)

    if(generate_random_map):
        for i in range(num_random_obstacles):
            add_obstacle(random.randint(2,width - rand_obstacle_size-2), random.randint(2,height - rand_obstacle_size-2), random.randint(2,rand_obstacle_size-2), random.randint(2,rand_obstacle_size-2), clearance,start_pose,goal_pose)
    else:
        for i in range(0,width,40):
            for j in range(0,height,40):
                add_obstacle(i,j,20,20)
    
    map.obstacle_tree = map1.obstacle_tree = map3.obstacle_tree = map2.obstacle_tree = KDTree(map.obstacle_list_padding)

    map2.nonobstacle_list = list(set([(x,y) for x in range(height) for y in range(width)])-set(map2.obstacle_list))
    map2.nonobstacle_tree = KDTree(map2.nonobstacle_list)

    def code(map):
        x_new = map_safe.Node(map.start.x,map.start.y)

        # map.display_map()
        
        time_list=[]

        initial_time=0
        num_iter=0
        while (num_iter < 100000):
        #while(map.euclidean_distance(x_new,map.goal) > 100):

            # if(map.solution_found):
            #     map.c_best = map.x_soln.sort()[0]
            
            x_rand = map.sample()

            seconds = time.time()

            if(map.collision_free(x_rand)!=0):
                nearest_node_found, x_nearest, cost = map.nearest_node(x_rand)

                if(nearest_node_found):
                    x_new, cost_new = map.steer(x_nearest, x_rand, cost)
                        
                    if (map.collision_free(x_nearest) != 0):
                        res = map.path_free(x_nearest, x_new)
                        if(res!=0):
                            if (res == 1):
                                edge = map_safe.Edge(x_new, x_nearest, cost_new)
                                x_new.parent = x_nearest
                                x_new.cost = x_new.parent.cost + cost_new

                                map.nodes.append(x_new)
                                map.freenodes.append([x_new.x,x_new.y])

                                edge.node_1 = x_new
                                edge.node_2 = x_nearest
                                edge.cost = cost_new

                                map.edges.append(edge)
                            else:
                                edge = map_safe.Edge(res, x_nearest, cost_new)
                                res.parent = x_nearest
                                res.cost = res.parent.cost + cost_new

                                map.nodes.append(res)
                                map.freenodes.append([res.x, res.y])

                                edge.node_1 = res
                                edge.node_2 = x_nearest
                                edge.cost = cost_new

                                map.edges.append(edge)
            curr_seconds = time.time()

            num_iter+=1
            # print(num_iter)
            
            initial_time=initial_time+curr_seconds-seconds
            time_list.append(initial_time)

            map.display_map()

        # map.solution_found = True

        # map.goal.parent = x_new
        # map.goal.cost = x_new.cost + map.euclidean_distance(x_new,map.goal)
        # map.freenodes.append(goal_pose)
        # map.nodes.append(map.goal)
        # cost_final = map.euclidean_distance(x_new,map.goal)
        # final_edge = map_safe.Edge(x_new, map.goal, cost_final)
        # map.edges.append(final_edge)

        # map.display_map(x_rand)

        # map.display_map(x_rand,best_path_found=True)
        # cv2.waitKey(0)

        return time_list
    
    time_list_norm=code(map3)
    time_list_safe=code(map1)
    time_list_unsafe=code(map2)
    time_list_safe_2=code(map3)

    x=[]
    length = min(len(time_list_norm), len(time_list_safe), len(time_list_unsafe))
    for i in range(length):
        x.append(i+1)

    plt.plot(x, time_list_norm[0:length], color='tab:red')
    plt.plot(x, time_list_safe[0:length], color='tab:blue')
    plt.plot(x, time_list_unsafe[0:length], color='tab:green')
    plt.plot(x, time_list_safe_2[0:length], color='tab:orange')
    # display the plot
    plt.show()

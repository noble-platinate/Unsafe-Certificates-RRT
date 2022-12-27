import random
import numpy as np
from sklearn.neighbors import KDTree
import scipy
import cv2

width = 1000
height = 1000
rand_obstacle_size = 50
img = np.ones((height, width, 3), np.uint8) * 255
obstacle_list=[]

def add_obstacle(x,y,width,height):
        for i in range(x,x+width):
            for j in range(y,y+height):
                img[i,j,0] = 0
                img[i,j,1] = 0
                img[i,j,2] = 0
                obstacle_list.append([i,j])

for i in range(1):
    add_obstacle(random.randint(2,width - rand_obstacle_size-2), random.randint(2,height - rand_obstacle_size-2), 50, 50)

tree = scipy.spatial.cKDTree(obstacle_list)
print(obstacle_list)
while(1):
    img = np.ones((height, width, 3), np.uint8) * 255
    for obstacle in obstacle_list:
        img[obstacle[0]][obstacle[1]] = (0, 0, 0)

    x = int(np.random.uniform(0, width))
    y = int(np.random.uniform(0, height))
    
    dist, node_near_idx = tree.query([[x, y]], k=1)
    print(node_near_idx)

    if(dist>0):
        a,b=obstacle_list[node_near_idx[0]]
        print(a,b)
        if(obstacle_list[node_near_idx[0]] in obstacle_list):
            print(True)
        img[a,b]=(0,0,0)
        cv2.circle(img, (b,a),
                   2, (100, 100, 10), -1)
        # cv2.circle(img, (x, y),
        #             int(dist[0][0]), (100, 0, 10), -1)

        cv2.imshow("self.map", img)
        cv2.waitKey(3000)
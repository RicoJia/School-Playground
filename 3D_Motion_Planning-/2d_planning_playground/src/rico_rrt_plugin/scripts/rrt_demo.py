from utils import load_map
from rrt import RRT, RRTConnect, RRTStar
import numpy as np
import cv2

if __name__=='__main__':
    done = False
    mp = load_map()
    while not done: 
        rrt_star = RRTStar(mp)
        done = rrt_star.plan()
        # rrt_connect = RRTConnect(mp)
        # done = rrt_connect.plan()
        # rrt = RRT(mp)
        # done = rrt.plan()
    

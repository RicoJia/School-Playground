#!/usr/bin/env python
'''
This file contains functions for adding obstacles to a numpy array plot.
Circular obstacles are drawn along the y axis. At each incremental position d along the y axis from the obstacle origin, a rectangular area is filled on the plot. The width of the
rectangle is cos(arcsin(d/r))
'''
import numpy as np
from PIL import Image
import cv2
from time import sleep
from itertools import count

WINDOW_NAME = "RRT demo"
ESC = 27

def generate_random_point(dim, upper_bounds, lower_bounds): 
    return np.random.uniform(np.array(lower_bounds), np.array(upper_bounds)).astype(int)

def draw_line(mp:np.ndarray, start, end, color=255): 
    # color = (0, 255, 0)
    thickness = 1
    image = cv2.line(mp, tuple(start), tuple(end), color, thickness)
    return image

def no_obstacle_in_line(mp: np.ndarray, start, end): 
    """
    Check for obstacles along a bresenham 8-connected line
    """
    zero_img = np.zeros_like(mp)
    zero_img = draw_line(zero_img, start, end)
    obstacles = np.logical_and(zero_img, mp)
    if np.count_nonzero(obstacles) == 0: 
        return True
    else: 
        return False
    
def is_within_boundary(row_num, column_num, pt):
    if pt[0]<row_num and pt[0]>=0:
        if pt[1]<column_num and pt[0]>=0:
            return True
    else:
        return False

def load_map():
    DARK_THRE = 100
    # PIC_FILE_PATH = './maze.jpeg'
    # PIC_FILE_PATH = './checker.png'
    PIC_FILE_PATH = './obstacle2.jpg'
    img = cv2.imread(PIC_FILE_PATH, cv2.IMREAD_GRAYSCALE)
    # Caution: in this case we want white space to be free space
    _, img = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY_INV)
    img = cv2.resize(img, (1000, 1000), interpolation = cv2.INTER_AREA)
    return img

class WindowManager(object):
    """Managing visualize windows"""
    def __init__(self, canvas_map):
        cv2.namedWindow(WINDOW_NAME)
        self.canvas_map = cv2.cvtColor(np.copy(canvas_map), cv2.COLOR_GRAY2BGR) 
        
    def __del__(self): 
        cv2.destroyAllWindows()
    def get_start_goal(self): 
        """
        Function that waits for two mouse clicks, which will be interpreted as start and goal
        """
        pts = []
        def click_pt(event, x_temp, y_temp, flags, params): 
            if event == cv2.EVENT_LBUTTONDOWN:
                cv2.circle(self.canvas_map, (x_temp, y_temp), 2, (255, 255, 0), thickness=2)
                pts.append(np.array([x_temp, y_temp]))

        cv2.setMouseCallback(WINDOW_NAME, click_pt) 

        while len(pts) < 2: 
            WindowManager.show_map(self.canvas_map, 10)
        self.canvas_map_final = np.copy(self.canvas_map)
        return pts

    def show_new_edges(self, new_pt_pairs: list, wait_time = 100, final_show = False): 
        """
        Function for showing newly added points and their parents
        """
        if final_show: 
            self.canvas_map = self.canvas_map_final
            color = (0, 255, 0)
        else: 
            color = (0, 0, 255)

        for new_pt_pair in new_pt_pairs: 
            if new_pt_pair:
                draw_line(self.canvas_map, new_pt_pair[0], new_pt_pair[1], color = color)
                cv2.circle(self.canvas_map, tuple(new_pt_pair[1]), 2, (0, 255, 255), thickness=1)
        key = WindowManager.show_map(self.canvas_map, wait_time)
        return key == ESC

    def show_all_edges(self, lookup: dict, wait_time = 100, final_show = False):
        """Function for showing all edges in a map, specifically for RRT Star

        :lookup: look up dictionary {coord (bytes): [parent_coords, ...] }
        :wait_time: live time for cv2 window
        :final_show: TODO
        :returns: TODO

        """
        self.canvas_map = np.copy(self.canvas_map_final)
        if final_show: 
            color = (0, 255, 0)
        else: 
            color = (0, 0, 255)

        for key, val in lookup.items(): 
            if val[0] is not None: 
                draw_line(self.canvas_map, np.frombuffer(key, dtype=int), val[0], color = color)
                cv2.circle(self.canvas_map, tuple(np.frombuffer(key, dtype=int)), 2, (0, 255, 255), thickness=1)
                # self.canvas_map = cv2.putText(self.canvas_map, f"{np.frombuffer(key, dtype=int)}",                                        tuple(np.frombuffer(key, dtype=int)), cv2.FONT_HERSHEY_PLAIN, 
                #                         1, (0, 255, 0), 1, cv2.LINE_AA)

        key = WindowManager.show_map(self.canvas_map, wait_time)
        return key == ESC

    @staticmethod
    def show_map(mp: np.ndarray, wait_time=0):
        cv2.imshow(WINDOW_NAME, mp)
        key = cv2.waitKey(wait_time)
        return key


# Kdtree
# Adopted from https://github.com/Vectorized/Python-KD-Tree/blob/master/kdtree.py
# Makes the KD-Tree for fast lookup: [[smaller half], [larger half], root]
def dist(p1, p2): 
    return np.linalg.norm(np.array(p1) - np.array(p2))

def make_kd_tree(points, dim, i=0):
    if len(points) > 1:
        points.sort(key=lambda x: x[i])
        i = (i + 1) % dim
        half = len(points) >> 1
        return [
            make_kd_tree(points[: half], dim, i),
            make_kd_tree(points[half + 1:], dim, i),
            points[half]
        ]
    elif len(points) == 1:
        return [None, None, points[0]]

# Adds a point to the kd-tree
def add_point(kd_node, point, dim, i=0):
    if kd_node is not None:
        dx = kd_node[2][i] - point[i]
        i = (i + 1) % dim
        # if dx < 0, go left. 
        for j, c in ((0, dx >= 0), (1, dx < 0)):
            if c and kd_node[j] is None:
                kd_node[j] = [None, None, point]
            elif c:
                add_point(kd_node[j], point, dim, i)

# k nearest neighbors
counter = count()
def get_knn(kd_node, point, k, dim, dist_func = dist, return_distances=True, i=0, heap=None):
    """
    [[dist, count, new_pt]...]
    """
    import heapq
    is_root = not heap
    if is_root:
        heap = []
    if kd_node is not None:
        dist = dist_func(point, kd_node[2])
        dx = kd_node[2][i] - point[i]
        if len(heap) < k:
            heapq.heappush(heap, (-dist, next(counter), kd_node[2]))
        elif dist < -heap[0][0]:
            heapq.heappushpop(heap, (-dist, next(counter), kd_node[2]))
        i = (i + 1) % dim
        # Goes into the left branch (kd_node[0]), and then the right branch if needed kd_node[1]
        for b in [dx < 0] + [dx >= 0] * (dx * dx < -heap[0][0]):
            get_knn(kd_node[b], point, k, dim, dist_func, return_distances, i, heap)
    if is_root:
        neighbors = sorted([(-h[0], h[1], h[2]) for h in heap], key=lambda x: x[0])
        return [(n[0], n[2]) for n in neighbors] if return_distances else [n[2] for n in neighbors]

# Simple and efficient implementation: 1. search the area that contains the target 2. Search the rest of the area. Quit once the the best possible (dx * dx) is greater than the existing best.
# TODO: might be buggy
def get_nearest(kd_node, point, dim, dist_func=dist, return_distances=True, i=0, best=None):
    if kd_node is not None:
        dist = dist_func(point, kd_node[2])
        dx = kd_node[2][i] - point[i]
        if not best:
            best = [dist, kd_node[2]]
        elif dist < best[0]:
            best[0], best[1] = dist, kd_node[2]
        i = (i + 1) % dim
        # Goes into the left/right branch (depending on dx), and then the other branch if needed
        # Rico: Smart. 
        for b in [dx < 0] + [dx >= 0] * (dx * dx < best[0]):
            get_nearest(kd_node[b], point, dim, dist_func, return_distances, i, best)
    return best if return_distances else best[1]


if __name__ == "__main__": 
    # kd tree test 
    from numpy import array
    kd_tree = [[[None, [None, None, array([355, 529])], array([334, 561])], None, array([354, 568])], [[None, [[None, [[None, [[None, None, array([481, 472])], None, array([462, 478])], array([444, 487])], None, array([435, 505])], array([422, 520])], None, array([405, 531])], array([394, 548])], [None, [[None, None, array([412, 590])], [None, [None, None, array([405, 628])], array([395, 611])], array([393, 592])], array([388, 573])], array([377, 557])], array([360, 549])]
    # 334, 561
    print(get_nearest(kd_tree, [334, 573], dim = 2))

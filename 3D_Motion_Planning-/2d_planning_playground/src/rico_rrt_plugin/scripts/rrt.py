import time
import numpy as np
from collections import defaultdict
from utils import draw_line, WindowManager, make_kd_tree, add_point, get_nearest, get_knn, generate_random_point, no_obstacle_in_line

class RRT():
    """Vanilla RRT"""
    def __init__(self, mp):
        """
        Initialize the map, and get start and goal out of the window_mgr
        """        
        self.mp = mp
        self.window_mgr = WindowManager(self.mp)
        self.start, self.goal = self.window_mgr.get_start_goal()

        if self.mp[tuple(self.start[::-1].astype(int))] != 0 or self.mp[tuple(self.goal[::-1].astype(int))] != 0: 
            raise RuntimeError("Invalid start or goal")

        self.dim = len(self.start)
        self.lower_bounds = np.zeros(self.dim)
        self.upper_bounds = self.mp.shape[0:2]
        # {coordinates: parent_coordinates}
        self.edge_len = 10.0
        self.lookup = {self.start.tobytes(): None}
        self.build_kd_tree()
        print(f"__init__ info: start - {self.start}, goal - {self.goal}")

    def plan(self): 
        """
        Main planning function for RRT. Visualization is added to show where each point is added
        """
        if not no_obstacle_in_line(self.mp, self.start, self.goal): 
            while True: 
                new_pt_pairs = self.plan_one_iteration()
                self.window_mgr.show_new_edges(new_pt_pairs, 1)
                if new_pt_pairs and no_obstacle_in_line(self.mp, new_pt_pairs[0][1], self.goal): 
                    self.lookup[self.goal.tobytes()] = new_pt_pairs[0][1]
                    break
        else: 
            self.add_point_to_rrt(self.start, self.goal, self.kd_tree)
        return self.window_mgr.show_new_edges(self.backtrack(self.goal), 10000, final_show=True)

    def build_kd_tree(self): 
        """
        Build a single kd tree
        """
        self.kd_tree = make_kd_tree([self.start], len(self.start))

    def get_valid_new_node_pair(self, random_pt, kd_tree): 
        """
        Return [nearest_node, new_node]. Empty if new_node -> random_pt has obstacle
        """
        dist, nearest_node = get_nearest(kd_tree, random_pt, self.dim)
        # we don't want the nearest node to be the same as the point - will contribute to redundancy. 
        if np.array_equal(nearest_node, random_pt): 
            return []
        if dist > self.edge_len: 
            random_pt = (nearest_node + (random_pt - nearest_node) * self.edge_len/dist).astype(int)
        if no_obstacle_in_line(self.mp, nearest_node, random_pt):
            return [nearest_node, random_pt]
        else: 
            return []

    def add_point_to_rrt(self, nearest_node, random_pt, kd_tree): 
        """
        Add random_pt to kd_tree and lookup if no obstacle to nearest_node
        """
        add_point(kd_tree, random_pt, self.dim)
        self.lookup[random_pt.tobytes()] = nearest_node

    def plan_one_iteration(self): 
        random_pt = generate_random_point(self.dim, self.lower_bounds, self.upper_bounds)
        new_pt_pair = self.get_valid_new_node_pair(random_pt, self.kd_tree)
        if new_pt_pair: 
            self.add_point_to_rrt(new_pt_pair[0], new_pt_pair[1], self.kd_tree)
            return [new_pt_pair]
        else: 
            return []

    def backtrack(self, start):
        """
        Back tracking function for all RRT methods
        """
        node = start
        path = []
        while True:
            next_node = self.lookup[node.tobytes()]
            if next_node is None:
                break
            path.append([node, next_node])
            node = next_node
        return path


class RRTStar(RRT): 
    """RRT star vanilla"""
    def __init__(self, mp) : 
        super().__init__(mp)
        # here look up is {current_coords: (parent_coords, dist_from_start)}
        self.lookup[self.start.tobytes()] = (None, 0)

    def plan(self): 
        """
        Main Plan function for RRT Star. Main difference
        """
        check_line_cnt = 0
        if not no_obstacle_in_line(self.mp, self.start, self.goal): 
            while True: 
                new_pt_pairs = self.plan_one_iteration()
                self.window_mgr.show_all_edges(self.lookup, 1)
                if new_pt_pairs and no_obstacle_in_line(self.mp, new_pt_pairs[0][1], self.goal): 
                    self.lookup[self.goal.tobytes()] = [new_pt_pairs[0][1], np.linalg.norm(self.goal - new_pt_pairs[0][1])]
                    break
        else: 
            self.add_point_to_rrt(self.start, self.goal, np.linalg.norm(self.goal - self.start), self.kd_tree)
        path = self.backtrack(self.goal)
        return self.window_mgr.show_all_edges(path, 10000, final_show = True)

    def plan_one_iteration(self):
        random_pt = generate_random_point(self.dim, self.lower_bounds, self.upper_bounds)
        new_pt_pair = self.get_valid_new_node_pair(random_pt, self.kd_tree)
        if new_pt_pair: 
            # Difference from RRT 1: find the best parent among k nearest neighbors
            new_pt = new_pt_pair[1]
            k = 10

            # the new nearest pt might be the new_pt itself because of distance normalization
            new_parent_found = False
            neighbors_dist = get_knn(self.kd_tree, new_pt, k, len(new_pt))
            neighbors_dist.sort(key = lambda x: x[0]) 
            for nearest_neighbor_dist in neighbors_dist: 
                edge_dist, new_parent = nearest_neighbor_dist
                if not np.array_equal(new_parent, new_pt) and no_obstacle_in_line(self.mp, new_pt, new_parent): 
                    self.add_point_to_rrt(new_parent, new_pt, edge_dist, self.kd_tree)
                    # # Difference from RRT 2: rewire
                    new_pt_dist_to_goal = self.lookup[new_pt.tobytes()][1]
                    for edge_dist, neighbor in neighbors_dist[1:]: 
                        if no_obstacle_in_line(self.mp, new_pt, neighbor): 
                            dist_alternative = edge_dist + new_pt_dist_to_goal
                            current_dist = self.lookup[neighbor.tobytes()][1]
                            if dist_alternative < current_dist: 
                                self.lookup[neighbor.tobytes()] = (new_pt, dist_alternative)
                    return [new_pt_pair]
            return []
        else: 
            return []

    def backtrack(self, start):
        """
        Back tracking function for all RRT methods
        """
        node = start
        path = dict()
        while True:
            next_node = self.lookup[node.tobytes()][0]
            if next_node is None:
                break
            # make it a [] because visualization requires it
            path[node.tobytes()] = [next_node]
            node = next_node
        return path
        
    def add_point_to_rrt(self, nearest_node, random_pt, edge_dist, kd_tree): 
        """
        Add random_pt to kd_tree and lookup if no obstacle to nearest_node
        """
        add_point(kd_tree, random_pt, self.dim)
        parent_dist = self.lookup[nearest_node.tobytes()][1]
        self.lookup[random_pt.tobytes()] = (nearest_node, parent_dist + edge_dist)



class RRTConnect(RRT):
    """RRT Connect Vanilla"""
    def __init__(self, mp):
        super(RRTConnect, self).__init__(mp)
        self.lookup[self.goal.tobytes()] = None

    def build_kd_tree(self): 
        self.kd_tree_start = make_kd_tree([self.start], len(self.start))
        self.kd_tree_goal = make_kd_tree([self.goal], len(self.goal))
        
    def plan(self):
        """
        Main planning function for RRTConnect
        """
        bridge = []
        if not no_obstacle_in_line(self.mp, self.start, self.goal):
            while True:
                new_pt_pairs = self.plan_one_iteration()
                self.window_mgr.show_new_edges(new_pt_pairs, 1)
                if new_pt_pairs and no_obstacle_in_line(self.mp, new_pt_pairs[0][1], new_pt_pairs[1][1]):
                    bridge = [new_pt_pairs[0][1], new_pt_pairs[1][1]]
                    break
        else:
            bridge = [self.start, self.goal]
        path = [bridge]
        path+=(self.backtrack(bridge[0]))
        path+=(self.backtrack(bridge[1]))
        return self.window_mgr.show_new_edges(path, 10000, final_show = True)

    def plan_one_iteration(self): 
        random_pt = generate_random_point(self.dim, self.lower_bounds, self.upper_bounds)
        new_pt_pair_s = self.get_valid_new_node_pair(random_pt, self.kd_tree_start)
        new_pt_pair_g = self.get_valid_new_node_pair(random_pt, self.kd_tree_goal)
        if new_pt_pair_s and new_pt_pair_g:
            self.add_point_to_rrt(new_pt_pair_s[0], new_pt_pair_s[1], self.kd_tree_start)
            self.add_point_to_rrt(new_pt_pair_g[0], new_pt_pair_g[1], self.kd_tree_goal)
            return [new_pt_pair_s, new_pt_pair_g]
        else:
            print("rejected")
            return []


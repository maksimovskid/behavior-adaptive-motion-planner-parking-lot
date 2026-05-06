"""
Path planning Sample Code with RRT with Dubins path

author: AtsushiSakai(@Atsushi_twi)

"""
import copy
import math
import os
import random
import sys
import timeit

import matplotlib.pyplot as plt
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../ReedsSheppPath/")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../RRT/")

try:
    from motion_planning.global_planning import reeds_shepp_path_planning
    from motion_planning.local_planning.rrt_1 import RRT
except ImportError:
    raise
#from new_map_with_obstacles import *

from motion_planning.path_interpolation.cubic_spline_planner import Spline2D

DEBUG_LOGGING = False


def debug_log(*args):
    """Print RRT diagnostics only when DEBUG_LOGGING is enabled."""
    if DEBUG_LOGGING:
        print(*args)


show_animation = False


# STEER_CHANGE_COST = 5.0  # steer angle change penalty cost
# STEER_COST = 1.0  # steer angle change penalty cost

class RRTReedsShepp(RRT):
    """
    Class for RRT planning with Dubins path
    """

    class Node(RRT.Node):
        """
        RRT Node
        """

        def __init__(self, x, y, yaw):
            super().__init__(x, y)
            self.cost = 0
            self.yaw = yaw
            self.path_yaw = []

    def __init__(self, start, goal, obstacle_list, rand_area,
                 goal_sample_rate=10,
                 max_iter=10,
                 ):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]

        """
        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1], goal[2])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list

        self.curvature = 0.18  # for dubins path
        self.goal_yaw_th = np.deg2rad(30)
        self.goal_xy_th = 1.0

    def planning(self, animation=True, search_until_max_iter=False):
        """
        execute planning

        animation: flag for animation on or off
        """
        start = timeit.default_timer()
        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd)

            if self.check_collision(new_node, self.obstacle_list):
                self.node_list.append(new_node)

            if animation and i % 5 == 0:
                self.plot_start_goal_arrow()
                self.draw_graph(rnd)

            if (not search_until_max_iter) and new_node:  # check reaching the goal
                last_index = self.search_best_goal_node()
                if last_index:
                    return self.generate_final_course(last_index)
            debug_log("Iter:", i, ", number of nodes:", len(self.node_list))
            stop = timeit.default_timer()
            debug_log('time: ', stop - start)

        debug_log("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index:
            debug_log('rrt path found')
            return self.generate_final_course(last_index)
        else:
            print("Cannot find path")

        return None

    def draw_graph(self, rnd=None):  # pragma: no cover
        """Draw the current Reeds-Shepp RRT tree for debugging/animation."""
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy) in self.obstacle_list:
            plt.plot(ox, oy, '.k')

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-2, 45, -2, 45])
        plt.axis("equal")
        plt.grid(True)
        self.plot_start_goal_arrow()
        plt.pause(0.01)

    def plot_start_goal_arrow(self):  # pragma: no cover
        """Draw start and goal arrows for the RRT debug plot."""
        reeds_shepp_path_planning.plot_arrow(
            self.start.x, self.start.y, self.start.yaw)
        reeds_shepp_path_planning.plot_arrow(
            self.end.x, self.end.y, self.end.yaw)

    def steer(self, from_node, to_node):
        """Connect two nodes using a Reeds-Shepp path segment."""

        px, py, pyaw, mode, course_lengths = reeds_shepp_path_planning.reeds_shepp_path_planning(
            from_node.x, from_node.y, from_node.yaw,
            to_node.x, to_node.y, to_node.yaw, self.curvature)

        if len(px) <= 0.1:  # cannot find a dubins path
            return None

      

        new_node = copy.deepcopy(from_node)
        new_node.x = px[-1]
        new_node.y = py[-1]
        new_node.yaw = pyaw[-1]

        new_node.path_x = px
        new_node.path_y = py
        new_node.path_yaw = pyaw
        new_node.cost += sum([abs(l) for l in course_lengths])
        new_node.parent = from_node

        return new_node

    def calc_new_cost(self, from_node, to_node):
        """Estimate path cost from an existing node to a candidate node."""

        _, _, _, _, course_lengths = reeds_shepp_path_planning.reeds_shepp_path_planning(
            from_node.x, from_node.y, from_node.yaw,
            to_node.x, to_node.y, to_node.yaw, self.curvature)

        return from_node.cost + sum([abs(l) for l in course_lengths]) 


    def get_random_node(self):
        """Sample a random pose, occasionally biasing toward the goal pose."""

        if random.randint(0, 1) > self.goal_sample_rate:
            rnd = self.Node(random.uniform(self.min_rand, self.max_rand),
                            random.uniform(self.min_rand, self.max_rand),
                            random.uniform(-math.pi/4, math.pi/4)   #changed angle from pi to  pi/4
                            )
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y, self.end.yaw)

        return rnd

    def search_best_goal_node(self):
        """Find the lowest-cost node that reaches the goal region."""

        goal_indexes = []    #check x,y 
        for (i, node) in enumerate(self.node_list):
            if self.calc_dist_to_goal(node.x, node.y) <= self.goal_xy_th:
                goal_indexes.append(i)
        # print("goal_indexes:", len(goal_indexes))

        # angle check
        # final_goal_indexes = []
        # for i in goal_indexes:
        #     if abs(self.node_list[i].yaw) <= self.goal_yaw_th:
        #         final_goal_indexes.append(i)
        # print("final_goal_indexes:", len(final_goal_indexes))


        if not goal_indexes:     #final goal indexes changed to  goal indexes
            return None

        min_cost = min([self.node_list[i].cost for i in goal_indexes])   #final goal indexes changed to  goal indexes
        for i in goal_indexes:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def generate_final_course(self, goal_index):
        """Trace parent links and return the final path from start to goal."""
        debug_log("generate final course")
        path = [[self.end.x, self.end.y, self.end.yaw]]
        node = self.node_list[goal_index]
        while node.parent:
            for (ix, iy, iyaw) in zip(reversed(node.path_x), reversed(node.path_y), reversed(node.path_yaw)):
                path.append([ix, iy, iyaw])
            node = node.parent
        path.append([self.start.x, self.start.y, self.start.yaw])
        return path


def main():
    """Run the original standalone RRT Reeds-Shepp demo."""
    print("Start " + __file__)
    # ====Search Path with RRT====
    obstacleList = [
        (0, 6), (1, 6), (2, 6), (3, 6), (4, 6), (5, 6), (6, 6), (7, 6), (8, 6), (9, 6), (10, 6), (11, 6), (12, 6),
        (13, 6), (14, 6),
        (15, 6), (16, 6), (17, 6), (18, 6), (19, 6), (20, 6), (21, 6), (22, 6), (23, 6), (24, 6), (25, 6), (26, 6),
        (27, 6),
        (28, 6), (29, 6), (30, 6), (31, 6), (32, 6), (33, 6), (34, 6), (35, 6), (36, 6), (37, 6), (38, 6), (39, 6),
        (40, 6),
        (41, 6), (41, 7), (41, 8), (41, 9), (41, 10), (41, 11), (41, 12), (41, 13), (41, 14), (41, 15), (41, 16),
        (41, 17),
        (41, 18), (41, 19), (41, 20), (41, 21), (41, 22), (41, 23), (41, 24), (41, 25), (41, 26), (41, 27), (41, 28),
        (41, 29),
        (41, 30), (41, 31), (41, 32), (41, 33), (41, 34), (41, 35), (41, 36), (41, 37), (41, 38), (41, 39), (41, 40),
        (40, 39),
        (39, 39), (38, 39), (37, 39), (37, 38), (37, 37), (37, 36), (37, 35), (37, 34), (36, 34), (35, 34), (34, 34),
        (33, 34),
        (32, 34), (31, 34), (30, 34), (29, 34), (28, 34), (27, 34), (26, 34), (25, 34), (24, 34), (23, 34), (22, 34),
        (21, 34),
        (20, 34), (19, 34), (18, 34), (17, 34), (16, 34), (15, 34), (14, 34), (13, 34), (12, 34), (11, 34), (10, 34),
        (9, 34),
        (8, 34), (7, 34), (6, 34), (5, 34), (4, 34), (3, 34), (2, 34), (1, 34), (0, 34), (0, 33), (0, 32), (0, 31),
        (0, 30),
        (0, 29), (0, 28), (0, 27), (0, 26), (0, 25), (0, 24), (0, 23), (0, 22), (0, 21), (0, 20), (0, 19), (0, 18),
        (0, 17),
        (0, 16), (0, 15), (0, 14), (0, 13), (0, 12), (0, 11), (0, 10), (0, 9), (0, 8), (0, 7), (0, 6), (0, 5), (0, 4),
        (0, 3), (0, 2),
        (0, 1), (0, 0), (0, 14), (1, 14), (2, 14), (3, 14), (4, 14), (5, 14), (6, 14), (7, 14), (8, 14),
        (9, 14), (10, 14), (11, 14), (12, 14), (13, 14), (14, 14), (15, 14), (16, 14), (17, 14), (18, 14), (19, 14),
        (20, 14),
        (21, 14), (22, 14), (23, 14), (24, 14), (25, 14), (26, 14), (27, 14), (28, 14), (29, 14), (30, 14), (31, 14),
        (32, 14),
        (33, 14), (33, 14), (33, 15), (33, 16), (33, 17), (33, 18), (33, 19), (33, 20), (33, 21), (33, 22), (33, 23),
        (33, 24), (33, 25), (33, 25), (33, 25), (32, 26), (31, 26), (30, 26), (29, 26), (28, 26), (27, 26), (26, 26),
        (25, 26), (24, 26), (23, 26), (22, 26), (21, 26), (20, 26), (19, 26), (18, 26), (17, 26), (16, 26), (15, 26),
        (14, 26),
        (13, 26), (12, 26), (11, 26), (10, 26), (9, 26), (8, 26), (7, 26), (6, 26), (5, 26), (4, 26), (3, 26), (2, 26),
        (1, 26), (0, 26),
        (0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0), (6, 0),
        # (0, 10), (1, 10), (2, 10), (3, 10), (4, 10), (5, 10), (6, 10), (7, 10), (8, 10), (9, 10), (10, 10), (11, 10),
        # (12, 10), (13, 10), (14, 10), (15, 10), (16, 10), (17, 10), (18, 10), (19, 10), (20, 10), (21, 10), (22, 10), (23, 10),
        # (24, 10), (25, 10), (26, 10), (27, 10), (28, 10), (29, 10), (30, 10), (31, 10), (32, 10), (33, 10), (34, 10), (35, 10),
        # (36, 10),    #till here down middle lane
        # (37, 11), (37, 12), (37, 13), (37, 14), (37, 15), (37, 16), (37, 17), (37, 18), (37, 19), (37, 20),
        # (37, 21), (37, 22), (37, 23), (37, 24), (37, 25), (37, 26), (37, 27), (37, 28), (37, 29),  #till here right middle lane
        # (0, 30), (1, 30), (2, 30), (3, 30), (4, 30), (5, 30), (6, 30), (7, 30), (8, 30), (9, 30), (10, 30), (11, 30),
        # (12, 30), (13, 30), (14, 30), (15, 30), (16, 30), (17, 30), (18, 30), (19, 30), (20, 30), (21, 30), (22, 30), (23, 30),
        # (24, 30), (25, 30), (26, 30), (27, 30), (28, 30), (29, 30), (30, 30), (31, 30), (32, 30), (33, 30), (34, 30), (35, 30),
        # (36, 30),    # till here upper middle lane
        (38, 34), (39, 34), (40, 34), (41, 34),
    ]  # [x,y]

    # Set Initial parameters
    start = [1.5, 8.0, np.deg2rad(0.0)]
    print('start ', start)
    goal = [11.499999999999975, 12.0, np.deg2rad(0.0)]
    print('goal', goal)

    rrt_reeds_shepp = RRTReedsShepp(start, goal, obstacleList, [0.0, 40.0])
    path = rrt_reeds_shepp.planning(animation=show_animation)
    if path is None:
        print("Cannot find path")
    else:
        print("path is found")
        print ("number of nodes in the final path: ", len(path))

    rrt_x = [state[0] for state in path][
            ::-1]  # since its plan reversed, this [::-1] reverses it to start from the first point again
    print('rrt_x points', rrt_x)
    print('length of rrt_x points', len(rrt_x))
    rrt_y = [state[1] for state in path][::-1]
    print('rrt_y points', rrt_y)

    h = np.diff(rrt_x)
    print('diff', h)


    rrt_x_rounded = [round(i, 2) for i in rrt_x]
    rrt_y_rounded = [round(i, 2) for i in rrt_y]

    print('rrt_x points rounded', rrt_x_rounded)
    print('rrt_y points rounded', rrt_y_rounded)

    x_spl_1 = rrt_x_rounded
    y_spl_1 = rrt_y_rounded

    sp_1 = Spline2D(x_spl_1, y_spl_1)
    s = np.arange(0, sp_1.s[-1], 0.1)
    # print(len(s) / 10)  # length of the path with the spline

    rx, ry, ryaw, rk1 = [], [], [], []
    for i_s in s:
        rk1.append(sp_1.calc_curvature(i_s))

    ck = rk1
    print('curvature', ck)


    # Draw final path
    if show_animation:  
        rrt_reeds_shepp.draw_graph()
        plt.plot([x for (x, y, yaw) in path], [y for (x, y, yaw) in path], '-r')
        plt.grid(True)
        plt.pause(0.001)
        plt.show()

        plt.subplots(1)
        plt.plot(s, ck, "-b", label="curvature")
        plt.grid(True)
        plt.legend()
        plt.xlabel("line length[m]")
        plt.ylabel("curvature [1/m]")


    if path and not show_animation: 
        rrt_reeds_shepp.draw_graph()
        plt.plot([x for (x, y, yaw) in path], [y for (x, y, yaw) in path],'.r')
        # print("List of x points:", [x for (x, y, yaw) in reversed(path)])        # for printing the points of the path 
        # print("List of y points:", [y for (x, y, yaw) in reversed(path)])
        # print("List of yaw points:", [yaw for (x, y, yaw) in reversed(path)])
        plt.grid(True)
        ax = plt.axes()
        ax.set_xlim(-5, 46)
        ax.set_ylim(-5, 41)
        ax.set_ylabel('vertical length (40 m)')
        ax.set_xlabel('horizontal length (41 m)')
        ax.set_title('parking lot with occupied spots')
        # for car in cars:
        #     ax.add_patch(car)
        #
        #
        # plt.plot(x1, y1, linestyle = '-', color ='k', lw=2.5)
        # plt.plot(x2, y2, linestyle = '-', color ='k', lw=2)
        # plt.plot(x3, y3, x4, y4, x5, y5, x6, y6, linestyle = '-', color ='y', lw=1.5)
        # plt.plot(x7, y7, linestyle = '--', color ='y', lw=1)

        # plt.subplots(1)
        # plt.plot(s, ck, "-r", label="curvature")
        # plt.grid(True)
        # plt.legend()
        # plt.xlabel("line length[m]")
        # plt.ylabel("curvature [1/m]")


        plt.show()



if __name__ == '__main__':
    main()

"""Cubic spline interpolation helpers used with Hybrid A* paths."""

import math
import numpy as np
import bisect

from motion_planning.global_planning.hybrid_a_star import hybrid_a_star_planning
from motion_planning.global_planning.a_star import dp_planning  # , calc_obstacle_map
from motion_planning.global_planning import reeds_shepp_path_planning as rs
from vehicle.car import move, check_car_collision, MAX_STEER, WB, plot_car

XY_GRID_RESOLUTION = 1.0  # [m]
YAW_GRID_RESOLUTION = np.deg2rad(15.0)  # [rad]




class Spline:
    u"""
    Cubic Spline class
    """

    def __init__(self, x, y):
        self.b, self.c, self.d, self.w = [], [], [], []

        self.x = x
        self.y = y

        self.nx = len(x)  # dimension of x
        h = np.diff(x)

        # calc coefficient c
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)
        #  print(self.c1)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * \
                (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def calc(self, t):
        u"""
        Calc position

        if t is outside of the input x, return None

        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return result

    def calcd(self, t):
        u"""
        Calc first derivative

        if t is outside of the input x, return None
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def calcdd(self, t):
        u"""
        Calc second derivative
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return result

    def __search_index(self, x):
        u"""
        search data segment index
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        u"""
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        #  print(A)
        return A

    def __calc_B(self, h):
        u"""
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / \
                h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
        #  print(B)
        return B


class Spline2D:
    u"""
    2D Cubic Spline class

    """

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = [math.sqrt(idx ** 2 + idy ** 2)
                   for (idx, idy) in zip(dx, dy)]
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        u"""
        calc position
        """
        x = self.sx.calc(s)
        y = self.sy.calc(s)

        return x, y

    def calc_curvature(self, s):
        u"""
        calc curvature
        """
        dx = self.sx.calcd(s)
        ddx = self.sx.calcdd(s)
        dy = self.sy.calcd(s)
        ddy = self.sy.calcdd(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2) * (3/2))
        return k

    def calc_yaw(self, s):
        u"""
        calc yaw
        """
        dx = self.sx.calcd(s)
        dy = self.sy.calcd(s)
        yaw = math.atan2(dy, dx)
        return yaw


def calc_spline_course(x, y, ds=0.1):
    """Sample a 2D cubic spline and return position, yaw, curvature, and arc length."""
    sp = Spline2D(x, y)
    s = np.arange(0, sp.s[-1], ds)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, s


def test_spline2d():
    """Run the original standalone spline plotting test."""
    print("Spline 2D test")

    import matplotlib.pyplot as plt

        # set obstable positions
    ox, oy = [], []
    for i in range(0, 42):     #outside boundaries
        ox.append(i)
        oy.append(0.0)
    for i in range(41):
        ox.append(41.0)
        oy.append(i)
    for i in range(42):
        ox.append(i)
        oy.append(40)
    for i in range(41):
        ox.append(0.0)
        oy.append(i)
    for i in range(4, 33):   #middle boundaries
        ox.append(i)
        oy.append(0)
    for i in range(14, 27):
        ox.append(33)
        oy.append(i)
    for i in range(0, 33):   
        ox.append(i)
        oy.append(26)
    for i in range(0, 34):   
        ox.append(i)
        oy.append(14)


    for i in range(0,42):   #down side parking boundary
        ox.append(i)
        oy.append(6)
    for i in range(0,37):   #upper side parking boundary
        ox.append(i)
        oy.append(34)
    for i in range(34, 41):   #parking spot boundary
        ox.append(37)
        oy.append(i)
    for i in range(0, 37):   #down middle lane defined as obstacle
        ox.append(i)
        oy.append(10.0)  
    for i in range(11, 30):   #right middle lane defined as obstacle
        ox.append(37.0)
        oy.append(i)    
    for i in range(0, 37):   #upper middle lane defined as obstacle
        ox.append(i)
        oy.append(30)    

    # Set Initial parameters
    start = [1.5, 8, np.deg2rad(0.0)]
    goal = [23.0, 32.0, np.deg2rad(180.0)]

    # plt.plot(ox, oy, ".k")
    # rs.plot_arrow(start[0], start[1], start[2], fc='g')
    # rs.plot_arrow(goal[0], goal[1], goal[2])

    plt.grid(True)
    plt.axis("equal")

    path = hybrid_a_star_planning(
        start, goal, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)

    cx = path.xlist
    cy = path.ylist
    cyaw = path.yawlist

    x = cx
    y = cy

    sp = Spline2D(x, y)
    s = np.arange(0, sp.s[-1], 0.1)
    print(len(s)/10)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    # flg, ax = plt.subplots(1)
    # plt.plot(x, y, "xb", label="input")
    # plt.plot(rx, ry, "-r", label="spline")
    # plt.grid(True)
    # plt.axis("equal")
    # plt.xlabel("x[m]")
    # plt.ylabel("y[m]")
    # plt.legend()

    # flg, ax = plt.subplots(1)
    # plt.plot(s, [math.degrees(iyaw) for iyaw in ryaw], "-r", label="yaw")
    # plt.grid(True)
    # plt.legend()
    # plt.xlabel("line length[m]")
    # plt.ylabel("yaw angle[deg]")

    flg, ax = plt.subplots(1)
    plt.plot(s, rk, "-r", label="curvature")
    plt.grid(True)
    plt.legend()
    plt.xlabel("path length[m]")
    plt.ylabel("curvature [1/m]")

    plt.show()



if __name__ == '__main__':
    test_spline2d()

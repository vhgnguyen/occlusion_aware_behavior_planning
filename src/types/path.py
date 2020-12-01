from cubic_spline import Spline2D
import numpy as np
import math 


class Path(object):

    def __init__(self):
        self._s = None
        self._ds = 0
        self._pt = []

    def setS(self, scenario):
        if scenario == 3:
            angles = np.arange(0.1, np.pi/2, 0.2)
            x = [2, 2, 2, 2]
            y = [-100, -50, -10, -4]
            c_x, c_y = [], []
            r = 2
            center = [4, -4]
            # draw circle turning
            for a in angles:
                x1 = center[0] - r * math.cos(a)
                y1 = center[1] + r * math.sin(a)
                x.append(x1)
                y.append(y1)
            x.extend([4, 10, 50, 100])
            y.extend([-2, -2, -2, -2])
            self._s = Spline2D(x, y)
        
        if scenario == 31:
            angles = np.arange(np.pi+0.01, 3*np.pi/2, 0.2)
            x = [-2, -2, -2, -2]
            y = [100, 50, 10, 4]
            c_x, c_y = [], []
            r = 6
            center = [4, 4]
            # draw circle turning
            for a in angles:
                x1 = center[0] + r * math.cos(a)
                y1 = center[1] + r * math.sin(a)
                x.append(x1)
                y.append(y1)
            x.extend([4, 10, 50, 100])
            y.extend([-2, -2, -2, -2])
            self._s = Spline2D(x, y)
        
        if scenario == 2:
            angles = np.arange(0.01, np.pi/2, 0.2)
            x = [2, 2, 2, 2]
            y = [-100, -50, -10, -4]
            c_x, c_y = [], []
            r = 6
            center = [-4, -4]
            # draw circle turning
            for a in angles:
                x1 = center[0] + r * math.cos(a)
                y1 = center[1] + r * math.sin(a)
                x.append(x1)
                y.append(y1)
            x.extend([-4, -10, -50, -100])
            y.extend([2, 2, 2, 2])
            self._s = Spline2D(x, y)

        s = np.arange(0, self._s.s[-1], 0.1)
        for i_s in s:
            ix, iy = self._s.calc_position(i_s)
            self._pt.append([ix, iy])

    def setStraightPath(self, startPose):
        pt1 = np.array([startPose.x_m, startPose.y_m]) - startPose.heading()
        pt2 = pt1 + startPose.heading()*100
        pt3 = pt1 + startPose.heading()*200

        x = [pt1[0], pt2[0], pt3[0]]
        y = [pt1[1], pt2[1], pt3[1]]
        self._s = Spline2D(x, y)
        s = np.arange(0, self._s.s[-1], 0.1)
        for i_s in s:
            ix, iy = self._s.calc_position(i_s)
            self._pt.append([ix, iy])

    def setDs(self, ds):
        self._ds = ds

    def updateDs(self, ds):
        self._ds = self._ds + ds
        self.updatePt()

    def updatePt(self):
        self._pt = []
        s = np.arange(self._ds, self._s.s[-1], 0.1)
        for i_s in s:
            ix, iy = self._s.calc_position(i_s)
            self._pt.append([ix, iy])

    def getDs(self, ds):
        ds = self._ds + ds
        ix, iy = self._s.calc_position(ds)
        iyaw = self._s.calc_yaw(ds)
        return ix, iy, iyaw

    def getCurrentDs(self):
        ix, iy = self._s.calc_position(self._ds)
        iyaw = self._s.calc_yaw(self._ds)
        return ix, iy, iyaw

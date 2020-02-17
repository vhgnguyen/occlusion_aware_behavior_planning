from matplotlib.patches import Ellipse, Polygon
import matplotlib.pyplot as plt

import numpy as np
import pose_functions as pfnc
import _param as param


class OtherVehicle:
    """
    Class define vehicle properties
    Params:
        _idx: vehicle index
        _u: current acceleration [m/s2] as input
        _p_pose(timestamp, pose): future path along the prediction horizon
        _p_u: predicted acceleration input along the prediction horizon
        _l_pose(timestamp, pose): traveled poses list
        _l_u(timestamp, u): recorded input during travel
        _is_moving: if vehicle is started or not
    """

    def __init__(self, idx, length, width):
        self._idx = idx
        self._length = length
        self._width = width
        self._u = None
        self._p_pose = {}
        self._p_u = {}
        self._l_pose = {}
        self._l_u = {}
        self._is_moving = False

    def start(self, startPose, u_in):
        """
        Call this function to start the vehicle
        Args:
            startPose: start sate of vehicle
            u_in: start acceleration
        Return:
            False if vehicle is already started, otherwise True
        """
        if self._is_moving:
            return False
        else:
            self._is_moving = True
            self._u = u_in
            self._l_u[startPose.timestamp_s] = u_in
            self._l_pose[startPose.timestamp_s] = startPose
        return True

    def getCurrentPose(self):
        # debug
        if not self._l_pose:
            print("Empty pose list")
            return
        return self._l_pose[max(self._l_pose)]

    def getCurrentTimestamp(self):
        # debug
        if not self._l_pose:
            print("Empty pose list")
            return
        return max(self._l_pose)

    def getPoseAt(self, timestamp_s):
        if timestamp_s not in self._l_pose:
            return None
        else:
            return self._l_pose[timestamp_s]

    def predict(self, l_u_in=None):
        """
        Predict the vehicle motion
        Args:
            l_u_in(timestamp_s, u_in): input list
                                           if None the current one is used
            p_timestamp_s: prediction time
        Return:
            Add the prediction acceleration and pose to prediction list

        """
        # debug:
        if not self._is_moving:
            print("No initial pose found")
            return
        if not l_u_in:
            print("Empty input list")
            return

        assert min(l_u_in) > self.getCurrentTimestamp()
        for u_timestamp, u_in in l_u_in.items():
            if not self._p_pose and not self._p_u:
                lastPose = self.getCurrentPose()
                self._p_pose = pfnc.updatePoseList(
                    lastPose=lastPose,
                    u_in=self._u,
                    nextTimestamp_s=u_timestamp
                    )
                self._p_u.update({u_timestamp: u_in})
            else:
                lastPose = self._getLastestPredictPose()
                new_p_pose = pfnc.updatePoseList(
                    lastPose=lastPose,
                    u_in=self._getLatestPredictInput(),
                    nextTimestamp_s=u_timestamp)
                self._p_pose.update(new_p_pose)
                self._p_u.update({u_timestamp: u_in})

    def update(self, next_timestamp_s):
        """
        Update vehicle state to next timestamp with predicted pose
        Args:
            next_timestamp_s: next travel time
        """
        if not self._p_pose or not self._p_u:
            print("No predicted state found")
            return

        new_l_pose = dict((k, self._p_pose[k])
                          for k in self._p_pose if k <= next_timestamp_s)
        new_u = dict((k, self._p_u[k])
                     for k in self._p_u if k <= next_timestamp_s)
        self._l_pose.update(new_l_pose)
        self._l_u.update(new_u)
        self._u = new_u[max(new_u)]

    def getPoly(self, timestamp_s):
        """
        Return the bounding polygon of vehicle
        """
        pose = self.getPoseAt(timestamp_s)
        if pose is None:
            print("No pose. is vehicle started?")
            return None
        return pfnc.rectangle(pose.x_m,
                              pose.y_m,
                              pose.yaw_rad,
                              self._length,
                              self._width)

    def plotAt(self, timestamp_s, ax=plt):
        if timestamp_s in self._l_pose:
            pose = self._l_pose[timestamp_s]
            ax.scatter(pose.x_m, pose.y_m, s=1, color='b')
            cov = pose.covLatLong
            ellipse = Ellipse([pose.x_m, pose.y_m],
                              width=np.sqrt(cov[0, 0])*2,
                              height=np.sqrt(cov[1, 1])*2,
                              angle=np.degrees(pose.yaw_rad),
                              facecolor=None,
                              edgecolor='blue',
                              alpha=0.4)
            ax.add_patch(ellipse)
        else:
            return

    def plot(self, maxTimestamp_s, ax=plt):
        for timestamp_s, pose in self._l_pose.items():
            if timestamp_s <= maxTimestamp_s:
                ax.scatter(pose.x_m, pose.y_m, s=1, color='b')
                cov = pose.covLatLong
                ellipse = Ellipse(
                    [pose.x_m, pose.y_m],
                    width=np.sqrt(cov[0, 0])*2,
                    height=np.sqrt(cov[1, 1])*2,
                    angle=np.degrees(pose.yaw_rad),
                    facecolor=None,
                    edgecolor='blue',
                    alpha=0.4)
                ax.add_patch(ellipse)
            if timestamp_s == maxTimestamp_s:
                poly = self.getPoly(timestamp_s)
                poly = Polygon(
                    poly, facecolor='cyan',
                    edgecolor='blue', alpha=0.7, label='other vehicle'
                )
                ax.add_patch(poly)

    # ------------------- Private function ---------------------

    def _getLastestPredictPose(self):
        # debug
        if not self._p_pose:
            print("Empty prediction pose list")
            return
        return self._p_pose[max(self._p_pose)]

    def _updateInput(self, u_in):
        if u_in > param._A_MAX:
            self._u = param._A_MAX
        elif u_in < param._A_MIN:
            self._u = param._A_MIN
        else:
            self._u = u_in

    def _getLatestPredictInput(self):
        # debug
        if not self._p_u:
            print("Empty prediction input list")
            return
        return self._p_u[max(self._p_u)]


class StaticObject(object):
    """
        Class define static object as polygon
        Param: UTM coordinate
            _idx: index of object
            _poly: (n,2) array vertices of object polygon
    """
    def __init__(self, idx, poly):
        self._idx = idx
        self._poly = poly
        self._center = np.mean(poly, axis=0)

    def plot(self, ax=plt):
        poly = Polygon(
            self._poly, facecolor='gray',
            edgecolor='black', alpha=1, label='static object'
            )
        ax.add_patch(poly)


class RoadBoundary(object):
    """
        Class define road boundary as lines between points
        Params: UTM coordinate in ^-> direction
            _idx: index (to classify with different vehicle)
            _left: (1,2,n) array points of left boundary
            _right: (1,2,n) array points of right boundary
    """
    def __init__(self, left=None, right=None, scenario=1):
        self._left = left
        self._right = right
        self._scenario = scenario

    def updateLeft(self, left):
        np.concatenate([self._left, left], axis=0)

    def updateRight(self, right):
        np.concatenate([self._right, right], axis=0)

    def toDict(self):
        return {'left': self._left, 'right': self._right}

    def plot(self, ax=plt):
        if self._scenario == 1:
            ax.plot([-30, 30], [-4, -4], 'k-')
            ax.plot([-30, 30], [4, 4], 'k-')

        if self._scenario == 2:
            ax.plot([-30, 10], [-4, -4], 'k-')
            ax.plot([-30, 12], [4, 4], 'k-')

            ax.plot([16, 30], [-4, -4], 'k-')
            ax.plot([18, 30], [4, 4], 'k-')

            ax.plot([12, 20], [4, 30], 'k-')
            ax.plot([18, 26], [4, 30], 'k-')

            ax.plot([10, 2], [-4, -30], 'k-')
            ax.plot([16, 8], [-4, -30], 'k-')

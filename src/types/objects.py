from matplotlib.patches import Ellipse, Polygon
import matplotlib.pyplot as plt

from pose import Pose, VehicleDynamic
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
        with UTM coordinate in ^-> direction
    """
    def __init__(self, scenario):
        self.left = None
        self.right = None
        self.lane = None
        self.setup(scenario)

    def setup(self, scenario):
        if scenario == 1:
            self.left = np.array([[[-100, 4], [100, 4]]])
            self.right = np.array([[[-100, -4], [100, -4]]])
            self.lane = np.array([[[-100, 0], [100, 0]]])

        if scenario == 2:
            self.left = np.array((
                [[-100, 4], [100, 4]],
                [[-4, -100], [-4, -4]]
            ))
            self.right = np.array((
                [[-100, -4], [-4, -4]],
                [[4, -4], [100, -4]],
                [[4, -100], [4, -4]]
            ))
            self.lane = np.array((
                [[-100, 0], [100, 0]],
                [[0, -100], [0, -4]]
            ))

        if scenario == 3:
            self.left = np.array((
                [[-100, 4], [-4, 4]],
                [[4, 4], [100, 4]],
                [[4, 100], [4, 4]],
                [[4, -100], [4, -4]]
            ))
            self.right = np.array((
                [[-100, -4], [-4, -4]],
                [[4, -4], [100, -4]],
                [[-4, 100], [-4, 4]],
                [[-4, -100], [-4, -4]]
            ))
            self.lane = np.array((
                [[-100, 0], [-4, 0]],
                [[4, 0], [100, -0]],
                [[0, 100], [0, 4]],
                [[0, -100], [0, -4]]
            ))

    def plot(self, ax=plt):
        nrRoad = self.lane.shape[0]
        for i in nrRoad:
            boundStyle = {'linestyle': '-', 'color': 'k'}
            plotLine(self.left[i], ax=ax, **boundStyle)
            plotLine(self.right[i], ax=ax, **boundStyle)
            laneStyle = {'linestyle': '--', 'color': 'k'}
            plotLine(self.lane[i], ax=ax, **laneStyle)


class Vehicle(object):

    def __init__(self, idx, length, width,
                 from_x_m, from_y_m, to_x_m, to_y_m,
                 covLong, covLat, vx_ms, startTime, isStop=False):
        self._idx = idx
        self._length = length
        self._width = width
        theta = np.arctan2(to_y_m-from_y_m, to_x_m-from_x_m)
        if isStop:
            self._u = pfnc.computeAccToStop(
                from_x_m=from_x_m, from_y_m=from_y_m,
                to_x_m=to_x_m, to_y_m=to_y_m, vx_ms=vx_ms)
        else:
            self._u = 0
        self._p_pose = {}
        startPose = Pose(
            x_m=from_x_m, y_m=from_y_m, yaw_rad=theta,
            covLatLong=np.diag([covLong, covLat]),
            vdy=VehicleDynamic(vx_ms, 0), timestamp_s=startTime)
        self._l_pose = {startPose.timestamp_s: startPose}

    def getLastPose(self):
        return self._l_pose[max(self._l_pose)]

    def getLastTimestamp(self):
        return max(self._l_pose)

    def getPoseAt(self, timestamp_s):
        if timestamp_s not in self._l_pose:
            return None
        else:
            return self._l_pose[timestamp_s]

    def predict(self, const_vx=True, pT=param._PREDICT_TIME):
        """
        Predict the vehicle motion from current state
        Args:
            const_vx: if True predict with constant velocity
                      else with current acceleration
        """
        self._p_pose = {}
        lastPose = self.getCurrentPose()
        nextTimestamp_s = round(self.getLastTimestamp() + pT, 3)
        if const_vx:
            self._p_pose = pfnc.updatePoseList(
                lastPose=lastPose,
                u_in=0,
                nextTimestamp_s=nextTimestamp_s
            )
        else:
            self._p_pose = pfnc.updatePoseList(
                lastPose=lastPose,
                u_in=self._u,
                nextTimestamp_s=nextTimestamp_s
            )

    def move(self, dT=param._dT):
        """
        Update vehicle state to next timestamp
        """
        lastPose = self.getCurrentPose()
        nextTimestamp_s = round(lastPose.timestamp_s + dT, 3)
        nextPose = pfnc.updatePose(
            lastPose=lastPose,
            u_in=self._u,
            dT=dT
            )
        self._l_pose.update({nextTimestamp_s: nextPose})

    def getPoly(self, timestamp_s):
        """
        Return the bounding polygon of vehicle
        """
        pose = self.getPoseAt(timestamp_s)
        if pose is None:
            print("No pose. is vehicle started?")
            return None
        return pfnc.rectangle(pose.x_m, pose.y_m, pose.yaw_rad,
                              self._length, self._width)

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


class Pedestrian(object):

    def __init__(self, idx, from_x_m, from_y_m, to_x_m, to_y_m,
                 covLong, covLat, vx_ms, startTime, isStop=False):
        self._idx = idx
        self._length = 1
        self._width = 1
        startTime = round(int(startTime/param._dT) * param._dT, 3)
        theta = np.arctan2(to_y_m-from_y_m, to_x_m-from_x_m)
        if isStop:
            s = np.sqrt((from_x_m-to_x_m)**2 + (from_y_m-to_y_m)**2)
            self._stopTimestamp_s = startTime + s / vx_ms
        else:
            self._stopTimestamp_s = 99999
        self._p_pose = {}
        startPose = Pose(
            x_m=from_x_m, y_m=from_y_m, yaw_rad=theta,
            covLatLong=np.diag([covLong, covLat]),
            vdy=VehicleDynamic(vx_ms, 0), timestamp_s=startTime)
        self._l_pose = {startPose.timestamp_s: startPose}

    def getLastPose(self):
        return self._l_pose[max(self._l_pose)]

    def getLastTimestamp(self):
        return max(self._l_pose)

    def getPoseAt(self, timestamp_s):
        if timestamp_s not in self._l_pose:
            return None
        else:
            return self._l_pose[timestamp_s]

    def predict(self, pT=param._PREDICT_TIME):
        """
        Predict the vehicle motion from current state
        Args:
            const_vx: if True predict with constant velocity
                      else with current acceleration
        """
        self._p_pose = {}
        lastPose = self.getCurrentPose()
        nextTimestamp_s = round(self.getLastTimestamp() + pT, 3)
        self._p_pose = pfnc.updatePoseList(
            lastPose=lastPose,
            u_in=0,
            nextTimestamp_s=nextTimestamp_s
        )

    def move(self, dT=param._dT):
        """
        Update vehicle state to next timestamp
        """
        lastPose = self.getCurrentPose()
        nextTimestamp_s = round(lastPose.timestamp_s + dT, 3)
        if nextTimestamp_s <= self._stopTimestamp_s:
            nextPose = pfnc.updatePose(
                lastPose=lastPose,
                u_in=self._u,
                dT=dT
                )
            self._l_pose.update({nextTimestamp_s: nextPose})

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


def plotLine(line, ax=plt, **kwargs):
    ax.plot([line[0][0], line[1][0]], [line[0][1], line[1][1]], **kwargs)


class PedestrianCross(object):

    def __init__(self, left, right, density):
        assert np.linalg.norm(left[0] - left[1]) == \
               np.linalg.norm(right[0] - right[1])
        self.left = left
        self.right = right
        self.density = density

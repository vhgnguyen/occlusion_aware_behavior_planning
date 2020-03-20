import numpy as np
import math


class Pose(object):
    """
    Class define vehicle pose (in global UTM coordinate)
    Args:
        x_m, y_m, yaw_rad: x,y position and heading
        cov_m: (2,2) matrix of pose uncertainty in vehicle coordinate
        vdy: current vehicle dynamic
        timestamp_s: time stamp in ms
    """
    def __init__(self, x_m, y_m, yaw_rad, covLatLong, vdy, timestamp_s):
        self.x_m = x_m
        self.y_m = y_m
        self.yaw_rad = yaw_rad
        self._s = np.sin(self.yaw_rad)
        self._c = np.cos(self.yaw_rad)
        self.vdy = vdy
        self.covLatLong = covLatLong
        self.covUtm = self._latlongToUTM()
        self.vxUtm = self._vxToUTM()
        self.timestamp_s = round(timestamp_s, 3)  # avoid floating error

    def _latlongToUTM(self):
        if self.covLatLong is None:
            return None
        else:
            T = np.array([[self._c, self._s], [-self._s, self._c]])
            return np.dot(np.dot(T, self.covLatLong), np.transpose(T))

    def _vxToUTM(self):
        return np.array([self._c, self._s]) * self.vdy.vx_ms

    def getRotation(self):
        return np.array([[self._c, self._s], [-self._s, self._c]])

    def getTranslation(self):
        return np.array([self.x_m, self.y_m])

    def heading(self):
        return np.array([self._c, self._s])


class VehicleDynamic(object):
    """
    Class define vehicle dynamic input
    Args:
        vx_ms: velocity in heading direction
        yaw_rads: yaw rate of vehicle
    """
    def __init__(self, vx_ms, dyaw_rads):
        self.vx_ms = max(vx_ms, 0)
        self.dyaw_rads = dyaw_rads

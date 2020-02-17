import numpy as np


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
        self.vdy = vdy
        self.covLatLong = covLatLong
        self.covUtm = self._latlongToUTM()
        self.vxUtm = self._vxToUTM()
        self.timestamp_s = round(timestamp_s, 3)  # avoid floating error

    def _latlongToUTM(self):
        if self.covLatLong is None:
            return None
        else:
            T = np.array([[np.cos(self.yaw_rad), np.sin(self.yaw_rad)],
                          [-np.sin(self.yaw_rad), np.cos(self.yaw_rad)]])
            return np.dot(np.dot(T, self.covLatLong), np.transpose(T))

    def _vxToUTM(self):
        vxUtm = np.array([np.cos(self.yaw_rad), np.sin(self.yaw_rad)])
        return vxUtm * self.vdy.vx_ms


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

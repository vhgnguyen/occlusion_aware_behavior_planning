import sys
import numpy as np

from environment import Environment
from pose import Pose, VehicleDynamic
from ego_car import EgoVehicle
from objects import Vehicle, StaticObject, Pedestrian, RoadBoundary
import _param as param

class Core(object):

    def __init__(self):
        self._egoCar = None
        self._env = Environment()
        self.timestamp_s = 0

    def addEgoVehicle(self, length, width, x_m, y_m, theta, cov_long, cov_lat,
        vx_ms, u_in, startTime):
        startPose = Pose(
            x_m=x_m, y_m=y_m, yaw_rad=theta,
            covLatLong=np.diag([cov_long, cov_lat]),
            vdy=VehicleDynamic(vx_ms, 0), timestamp_s=startTime)
        self._egoCar = EgoVehicle(
            length=length, width=width, env=self._env,
            startPose=startPose, u_in=u_in)
        print("Ego vehicle added.")

    def addOtherVehicle(self, length, width, x_m, y_m, to_x_m, to_y_m,
        cov_long, cov_lat, vx_ms, startTime, isStop=False):

        otherCar = Vehicle(
            idx=self._env.countVehicle()+1,
            length=length, width=width,
            from_x_m=x_m, from_y_m=y_m,
            to_x_m=to_x_m, to_y_m=to_y_m,
            covLong=cov_long, covLat=cov_lat,
            vx_ms=vx_ms, startTime=startTime, isStop=isStop)
        self._env.addVehicle(otherCar)
        print("New vehicle added. Vehicle count: {:}".format(self._env.countVehicle()))

    def addPedestrian(self, x_m, y_m, to_x_m, to_y_m,
        cov_long, cov_lat, vx_ms, startTime, isStop=False):

        pedestrian = Pedestrian(
            idx=self._env.countPedestrian()+1,
            from_x_m=x_m, from_y_m=y_m,
            to_x_m=to_x_m, to_y_m=to_y_m,
            covLong=cov_long, covLat=cov_lat,
            vx_ms=vx_ms, startTime=startTime, isStop=isStop)
        self._env.addPedestrian(pedestrian)
        print("New pedestrian added. Pedestrian count: {:}".format(self._env.countPedestrian()))

    def addStaticObject(self, staticObject):
        self._env.addStaticObject(staticObject)

    def setScenario(self, nr):
        self._env.setScenario(nr)

    def move(self, dT=param._dT):
        self._egoCar._move()
        self._env.move()
    
    def reset(self):
        self._egoCar = None
        self._env = Environment()

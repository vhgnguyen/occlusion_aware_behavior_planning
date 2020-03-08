import sys
import numpy as np

from environment import Environment
from pose import Pose, VehicleDynamic
from ego_car import EgoVehicle
from objects import OtherVehicle, StaticObject
import _param as param

class Core(object):

    def __init__(self):
        self._egoCar = None
        self._env = Environment()

    def addEgoVehicle(
        self, length, width,
        x_m, y_m, theta,
        cov_long, cov_lat,
        vx_ms, starTime,
        
        ):
        self._egoCar = egoCar
    
    def addOtherVehicle(self, otherCar):
        self._env.addVehicle(otherCar)
    
    def addPedestrian(self, pedestrian):
        self._env.addPedestrian(pedestrian)
    
    def addStaticObject(self, staticObject):
        self._env.addStaticObject(staticObject)

    def setScenario(self, nr):
        self._env.setScenario(nr)

    def move(self, dT=param._dT):
        self._egoCar._move()
        self._env.move()

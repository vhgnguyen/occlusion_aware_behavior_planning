import numpy as np


class Core(object):

    def __init__(self, env):
        self._egoCar = None
        self._env = env

    def addEgoVehicle(self, egoCar):
        self._egoCar = egoCar
    
    def addOtherVehicle(self, otherCar):
        self._env.addVehicle(otherCar)
    
    def addPedestrian(self, pedestrian):
        self._env.addPedestrian(pedestrian)

    def setScenario(self, nr):
        self._env.setScenario(nr)

    def update(self, timestamp_s):
        
        


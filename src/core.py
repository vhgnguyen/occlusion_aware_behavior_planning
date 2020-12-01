import numpy as np

from environment import Environment
from pose import Pose, VehicleDynamic
from ego_car import EgoVehicle
from objects import Vehicle, Pedestrian, StaticObject
import _param as param


class Core(object):
    """
        Interface to agorithm 
    """

    def __init__(self):
        self._egoCar = None
        self._env = Environment()
        self.timestamp_s = 0
        self.simulationTime = param._SIMULATION_TIME
        self.dT = param._dT

    """ Run functions """
    def move(self):
        if (self._egoCar is not None and
           self.timestamp_s + self.dT <= self.simulationTime):
            if self.timestamp_s == 0:
                self._egoCar.start()
            self._egoCar.optimizeState(
                dT=param._dT, predictStep=param._PREDICT_STEP,
                predictTime=param._PREDICT_TIME)
            self._env.move(
                egoPose=self._egoCar.getCurrentPose(),
                currentTime=self.timestamp_s, dT=param._dT)
            self.timestamp_s = round(self.timestamp_s + self.dT, 3)
            return True
        else:
            return False

    def reset(self):
        self._egoCar = None
        self._env = Environment()
        self.timestamp_s = 0

    def restart(self):
        if self._egoCar is None:
            return
        self.timestamp_s = 0
        self._egoCar.restart()
        self._env.restart()

    def replay(self):
        if self._egoCar is None:
            return
        # bool for replay

    def getCurrentTime(self):
        return self.timestamp_s

    def getSimulationTime(self):
        return self.simulationTime

    def updateSimulationTime(self, sT):
        self.simulationTime = sT

    def updateTimeStep(self, dT):
        self.dT = dT

    """ Add elements functions """

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
        self.restart()

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
        print("Vehicle count: {:}".format(self._env.countVehicle()))

    def addPedestrian(self, x_m, y_m, to_x_m, to_y_m,
                      cov_long, cov_lat, vx_ms, startTime, isStop=False):

        pedestrian = Pedestrian(
            idx=self._env.countPedestrian()+1,
            from_x_m=x_m, from_y_m=y_m,
            to_x_m=to_x_m, to_y_m=to_y_m,
            covLong=cov_long, covLat=cov_lat,
            vx_ms=vx_ms, startTime=startTime, isStop=isStop)
        self._env.addPedestrian(pedestrian)
        print("Pedestrian count: {:}".format(self._env.countPedestrian()))

    def addStaticObject(self, x_m, y_m, length, width):

        obs = StaticObject(
            idx=self._env.countStaticObject()+1,
            poly=np.array([[x_m, y_m], [x_m + length, y_m],
                          [x_m + length, y_m + width],
                          [x_m, y_m + width]]))
        self._env.addStaticObject(obs)

    """ Export functions """

    # ego vehicle

    def getCurrentEgoPoly(self):
        if self._egoCar is None:
            return None
        return self._egoCar.getCurrentPoly()

    def getCurrentPath(self):
        if self._egoCar is None:
            return None
        return self._egoCar._path._pt

    def getPredictEgo(self):
        if self._egoCar is None:
            return None
        return self._egoCar.exportPredictState()

    def getCurrentEgoPos(self):
        if self._egoCar is None:
            return None
        return [self._egoCar.getCurrentPose().x_m,
                self._egoCar.getCurrentPose().y_m,
                self._egoCar.getCurrentPose().yaw_rad]

    def getCurrentEgoHeading(self):
        if self._egoCar is None:
            return None
        return self._egoCar.getCurrentPose().yaw_rad

    def getCurrentFOV(self):
        if self._egoCar is None:
            return None
        else:
            return self._egoCar._fov

    def getCurrentVelocity(self):
        if self._egoCar is None:
            return None
        return round(self._egoCar.getCurrentLongtitudeVelocity(), 3)

    def getCurrentAcceleration(self):
        if self._egoCar is None:
            return None
        return round(self._egoCar.getCurrentLongtitudeAcceleration(), 3)

    def getTravelDistance(self):
        if self._egoCar is None:
            return None
        return self._egoCar.getTravelDistance()
    
    def getCurrentState(self):
        if self._egoCar is None:
            return None
        return self._egoCar.getCurrentState()

    # environment

    def exportCurrentPedestrian(self):
        pedesList = []
        for pedes in self._env._l_pedestrian:
            if pedes.getCurrentTimestamp() == self.timestamp_s:
                c = pedes.exportCurrent()
                p = pedes.exportPredict()
                pedesList.append({'c': c, 'p': p})
        return pedesList

    def exportCurrentVehicle(self):
        vehList = []
        for vehicle in self._env._l_vehicle:
            if vehicle.getCurrentTimestamp() == self.timestamp_s:
                c = vehicle.exportCurrent()
                p = vehicle.exportPredict()
                vehList.append({'c': c, 'p': p})
        return vehList

    def exportHypoPedestrian(self):
        hypoList = []
        for pedes in self._env._l_hypoPedes:
            c = pedes.exportCurrent()
            p = pedes.exportPredict()
            hypoList.append({'c': c, 'p': p})
        return hypoList

    def exportHypoVehicle(self):
        hypoList = []
        for vehicle in self._env._l_hypoVehicle:
            c = vehicle.exportCurrent()
            p = vehicle.exportPredict()
            hypoList.append({'c': c, 'p': p})
        return hypoList

    # plot and safe

    def plotDynamic(self, safeVelocity=False):
        if self._egoCar is None:
            return
        self._egoCar.plotDynamic(safeVelocity)

    def plotDynamicDistance(self, safeVelocity=False):
        if self._egoCar is None:
            return
        self._egoCar.plotDynamicDistance(safeVelocity)

    def plotRisk(self):
        if self._egoCar is None:
            return
        self._egoCar.plotPassedCost()

    def saveDynamic(self, path, fileName):
        if self._egoCar is None:
            return
        self._egoCar.saveDynamic(path, fileName)

    def saveDynamicDistance(self, path, fileName):
        if self._egoCar is None:
            return
        self._egoCar.saveDynamicDistance(path, fileName)

    def saveRisk(self, path, fileName):
        if self._egoCar is None:
            return
        self._egoCar.saveRisk(path, fileName)

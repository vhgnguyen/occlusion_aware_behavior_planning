import matplotlib.pyplot as plt
import numpy as np

from pose import Pose, VehicleDynamic
from objects import OtherVehicle, StaticObject, RoadBoundary
import _param as param


class Environment(object):
    """
        Class define environment
    """
    def __init__(self):
        self._l_road = []
        self._l_staticObject = []
        self._l_vehicle = []
        self._l_updateObject = []

    def addRoadBoundary(self, roadBoundary):
        self._l_road.append(roadBoundary)

    def addVehicle(self, otherVehicle):
        self._l_vehicle.append(otherVehicle)

    def addStaticObject(self, staticObject):
        self._l_staticObject.append(staticObject)

    def addMoveObject(self, moveObject):
        self._l_moveObject.append(moveObject)

    def updateAt(self, x_m, y_m, timestamp_s, radius=param._SCAN_RADIUS):
        """
        Get environment around a given position
        """
        self._l_updateObject = []
        # find static object
        currentPos = np.array([x_m, y_m])
        for sObj in self._l_staticObject:
            d2ego = np.linalg.norm(sObj._poly - currentPos, axis=1)
            if ((d2ego) < radius).any():
                self._l_updateObject.append(sObj)

        # find vehicle
        for veh in self._l_vehicle:
            vehPose = veh.getPoseAt(timestamp_s)
            if vehPose is not None:
                vehPos = np.array([vehPose.x_m, vehPose.y_m])
                if np.linalg.norm(vehPos - currentPos) < radius:
                    self._l_updateObject.append(veh)

    def update(self, x_m, y_m, timestamp_s, from_timestamp, radius=param._SCAN_RADIUS):
        """
        Get environment around a given position
        """
        currentPos = np.array([x_m, y_m])
        l_updateObject = []

        # find static object
        for sObj in self._l_staticObject:
            d2ego = np.linalg.norm(sObj._poly - currentPos, axis=1)
            if ((d2ego) < radius).any():
                l_updateObject.append(sObj)

        # find vehicle
        for veh in self._l_vehicle:
            # if object doesn't appear at current time
            if veh.getPoseAt(from_timestamp) is None:
                continue
            vehPose = veh.getPoseAt(timestamp_s)
            if vehPose is not None:
                vehPos = np.array([vehPose.x_m, vehPose.y_m])
                if np.linalg.norm(vehPos - currentPos) < radius:
                    l_updateObject.append(veh)
        return l_updateObject

    def plot(self, timestamp_s, plotHistory=False, ax=plt):
        """
        Plot the environment at given timestamp
        """
        for obj in self._l_staticObject:
            obj.plot(ax=ax)
        for road in self._l_road:
            road.plot(ax=ax)
        if not plotHistory:
            for veh in self._l_vehicle:
                veh.plotAt(timestamp_s, ax)
        else:
            for veh in self._l_vehicle:
                veh.plot(maxTimestamp_s=timestamp_s, ax=ax)

    def setupScenario(self, scenario):

        if scenario == 1:
            # first vehicle
            other1 = OtherVehicle(1, 4, 2)
            startPose1 = Pose(
                x_m=30, y_m=1, yaw_rad=-np.pi,
                covLatLong=np.array([[0.3, 0.0], [0.0, 0.2]]),
                vdy=VehicleDynamic(5, 0), timestamp_s=0)
            other1.start(startPose=startPose1, u_in=0.5)
            other1.predict(l_u_in={param._SIMULATION_TIME: 0})
            other1.update(param._SIMULATION_TIME)
            self.addVehicle(other1)
            # static object
            obs1 = StaticObject(
                idx=1,
                poly=np.array([[-10, -5], [-5, -5], [-5, -15], [-10, -15]]))
            self.addStaticObject(obs1)
            # road boundary
            road1 = RoadBoundary(scenario=1)
            self.addRoadBoundary(road1)

        if scenario == 2:
            # first vehicle
            other1 = OtherVehicle(idx=1, length=4, width=2)
            startPose1 = Pose(
                x_m=20, y_m=1, yaw_rad=-np.pi,
                covLatLong=np.array([[0.3, 0.0], [0.0, 0.1]]),
                vdy=VehicleDynamic(10, 0), timestamp_s=0)
            other1.start(startPose=startPose1, u_in=1)
            other1.predict(l_u_in={param._SIMULATION_TIME: 0})
            other1.update(param._SIMULATION_TIME)
            # self.addVehicle(other1)
            # second vehicle
            other2 = OtherVehicle(idx=2, length=4, width=2.5)
            startPose2 = Pose(
                x_m=20, y_m=20, yaw_rad=4.4,
                covLatLong=np.array([[0.3, 0.0], [0.0, 0.2]]),
                vdy=VehicleDynamic(8, 0), timestamp_s=4)
            other2.start(startPose=startPose2, u_in=0)
            other2.predict(l_u_in={param._SIMULATION_TIME: 0})
            other2.update(param._SIMULATION_TIME)
            self.addVehicle(other2)
            # pedestrian
            pedes = OtherVehicle(idx=3, length=1, width=1)
            startPose3 = Pose(
                x_m=-3, y_m=-7, yaw_rad=np.pi/2 - 0.5,
                covLatLong=np.array([[1, 0.0], [0.0, 0.5]]),
                vdy=VehicleDynamic(3, 0), timestamp_s=2)
            pedes.start(startPose=startPose3, u_in=0)
            pedes.predict(l_u_in={param._SIMULATION_TIME: 0})
            pedes.update(param._SIMULATION_TIME)
            self.addVehicle(pedes)
            # static object
            obs1 = StaticObject(
                idx=1,
                poly=np.array([[-20, -10], [-20, -6], [-15, -4],
                               [-5, -4], [-5, -15], [-15, -15]]))
            self.addStaticObject(obs1)
            # road boundary
            road1 = RoadBoundary(scenario=2)
            self.addRoadBoundary(road1)

        if scenario == 3:
            # pedestrian
            pedes = OtherVehicle(idx=3, length=1, width=1)
            startPose3 = Pose(
                x_m=-3, y_m=-7, yaw_rad=np.pi/2 - 0.5,
                covLatLong=np.array([[1, 0.0], [0.0, 0.5]]),
                vdy=VehicleDynamic(3, 0), timestamp_s=4)
            pedes.start(startPose=startPose3, u_in=0)
            pedes.predict(l_u_in={param._SIMULATION_TIME: 0})
            pedes.update(param._SIMULATION_TIME)
            self.addVehicle(pedes)
            # static object
            obs1 = StaticObject(
                idx=1,
                poly=np.array([[-20, -10], [-20, -6], [-15, -4], [-5, -4], [-5, -15], [-15, -15]]))
            self.addStaticObject(obs1)

            obs2 = StaticObject(
                idx=2,
                poly=np.array([[0, 3], [7, 3], [7, 7], [0, 7]]))
            # self.addStaticObject(obs2)

            obs3 = StaticObject(
                idx=3,
                poly=np.array([[0, -7], [0, -5], [7, -5], [7, -8]]))
            # self.addStaticObject(obs3)

            # road boundary
            road1 = RoadBoundary(scenario=2)
            self.addRoadBoundary(road1)

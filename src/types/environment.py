import matplotlib.pyplot as plt
import numpy as np

from pose import Pose, VehicleDynamic
from objects import OtherVehicle, StaticObject, RoadBoundary, PedestrianCross
import _param as param


class Environment(object):
    """
        Class define environment
    """
    def __init__(self):
        self._l_road = 0
        self._l_staticObject = []
        self._l_vehicle = []
        self._l_pedestrian = []
        self._l_cross = []

    def countPedestrian(self):
        return len(self._l_pedestrian)

    def countVehicle(self):
        return len(self._l_vehicle)

    def countStaticObject(self):
        return len(self._l_staticObject)

    def addRoadBoundary(self, roadBoundary):
        self._l_road.append(roadBoundary)

    def addVehicle(self, otherVehicle):
        self._l_vehicle.append(otherVehicle)

    def addStaticObject(self, staticObject):
        self._l_staticObject.append(staticObject)

    def addPedestrian(self, pedestrian):
        self._l_pedestrian.append(pedestrian)

    def addPedestrianCross(self, cross):
        self._l_cross.append(cross)

    def move(self, currentTime, dT=param._dT):
        """
        Move objects in environment to next step
        """
        for vehicle in self._l_vehicle:
            if currentTime >= vehicle._startTime:
                vehicle.move()
        for pedestrian in self._l_pedestrian:
            if currentTime >= pedestrian._startTime:
                pedestrian.move()

    def updateAt(self, x_m, y_m, timestamp_s, radius=param._SCAN_RADIUS):
        """
        Get environment around a given position
        """
        l_vehicle = []
        l_object = []
        l_pedestrian = []

        # find static object
        currentPos = np.array([x_m, y_m])
        for sObj in self._l_staticObject:
            d2ego = np.linalg.norm(sObj._poly - currentPos, axis=1)
            if ((d2ego) < radius).any():
                l_object.append(sObj)

        # find vehicle
        for veh in self._l_vehicle:
            vehPose = veh.getPoseAt(timestamp_s)
            if vehPose is not None:
                vehPos = np.array([vehPose.x_m, vehPose.y_m])
                if np.linalg.norm(vehPos - currentPos) < radius:
                    l_vehicle.append(veh)

        # find pedestrian
        for pedes in self._l_pedestrian:
            pedesPose = pedes.getPoseAt(timestamp_s)
            if pedesPose is not None:
                pedesPos = np.array([pedesPose.x_m, pedesPose.y_m])
                if np.linalg.norm(pedesPos - currentPos) < radius:
                    l_pedestrian.append(pedes)
                    
        l_update = {'vehicle': l_vehicle,
                    'staticObject': l_object,
                    'pedestrian': l_pedestrian}

        return l_update

    def update(self, x_m, y_m, timestamp_s, from_timestamp,
               radius=param._SCAN_RADIUS):
        """
        Get environment around a given position
        """
        l_vehicle = []
        l_object = []
        l_pedestrian = []

        currentPos = np.array([x_m, y_m])

        # find static object
        for sObj in self._l_staticObject:
            d2ego = np.linalg.norm(sObj._poly - currentPos, axis=1)
            if ((d2ego) < radius).any():
                l_object.append(sObj)

        # find vehicle
        for veh in self._l_vehicle:
            # if object doesn't appear at current time
            vehPose = veh.getPoseAt(from_timestamp)
            if vehPose is not None:
                vehPos = np.array([vehPose.x_m, vehPose.y_m])
                if np.linalg.norm(vehPos - currentPos) < radius:
                    l_vehicle.append(veh)

        # find pedestrian
        for pedes in self._l_pedestrian:
            pedesPose = pedes.getPoseAt(from_timestamp)
            if pedesPose is not None:
                pedesPos = np.array([pedesPose.x_m, pedesPose.y_m])
                if np.linalg.norm(pedesPos - currentPos) < radius:
                    l_pedestrian.append(pedes)

        l_update = {'vehicle': l_vehicle,
                    'staticObject': l_object,
                    'pedestrian': l_pedestrian}

        return l_update

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

    def setScenario(self, scenario):
        self._l_road = 0
        self._l_staticObject = []
        self._l_vehicle = []
        self._l_pedestrian = []
        self._l_cross = []

        if scenario == 1:
            # static object
            obs1 = StaticObject(
                idx=1,
                poly=np.array([[-40, -20], [-10, -20], [-10, -10],
                               [-20, -5], [-40, -5]]))
            self.addStaticObject(obs1)

            # pedestrian cross
            cross1 = PedestrianCross(
                left=np.array([[10, -20], [10, 20]]),
                right=np.array([[15, -20], [15, 20]]),
                density=0.5
            )
            self.addPedestrianCross(cross1)

        elif scenario == 2:
            # static object
            obs1 = StaticObject(
                idx=1,
                poly=np.array([[-40, -20], [-10, -20], [-10, -10],
                               [-20, -5], [-40, -5]]))
            self.addStaticObject(obs1)

            obs2 = StaticObject(
                idx=2,
                poly=np.array([[8, -20], [30, -20], [30, -5], [8, -5]]))
            self.addStaticObject(obs2)

            # pedestrian cross
            cross1 = PedestrianCross(
                left=np.array([[-10, -20], [-10, 20]]),
                right=np.array([[-4, -20], [-4, 20]]),
                density=0.8
            )
            self.addPedestrianCross(cross1)

        elif scenario == 3:
            # static object
            obs1 = StaticObject(
                idx=1,
                poly=np.array([[-40, -20], [-10, -20], [-10, -10],
                               [-20, -5], [-40, -5]]))
            self.addStaticObject(obs1)

            obs2 = StaticObject(
                idx=2,
                poly=np.array([[8, -20], [30, -20], [30, -5], [8, -5]]))
            self.addStaticObject(obs2)

            obs3 = StaticObject(
                idx=3,
                poly=np.array([[-40, 8], [-20, 8], [-10, 15], [-10, 20], [-40, 20]]))
            self.addStaticObject(obs3)

            # pedestrian cross
            cross1 = PedestrianCross(
                left=np.array([[-10, -20], [-10, 20]]),
                right=np.array([[-4, -20], [-4, 20]]),
                density=0.8
            )
            self.addPedestrianCross(cross1)

        # road boundary
        self._l_road = RoadBoundary(scenario=scenario)

    def setupScenario1(self, scenario):

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
                vdy=VehicleDynamic(3, 0), timestamp_s=4)
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

            obs2 = StaticObject(
                idx=2,
                poly=np.array([[0, 4], [10, 7], [10, 10], [0, 10]]))
            self.addStaticObject(obs2)

            obs3 = StaticObject(
                idx=3,
                poly=np.array([[5, -10], [5, -3], [10, -3], [8, -10]]))
            self.addStaticObject(obs3)
            # road boundary
            road1 = RoadBoundary(scenario=2)
            self.addRoadBoundary(road1)

        if scenario == 3:
            # pedestrian
            pedes = OtherVehicle(idx=3, length=1, width=1)
            startPose3 = Pose(
                x_m=-3, y_m=-5, yaw_rad=np.pi/2,
                covLatLong=np.array([[1, 0.0], [0.0, 0.5]]),
                vdy=VehicleDynamic(4, 0), timestamp_s=4.6)
            pedes.start(startPose=startPose3, u_in=0)
            pedes.predict(l_u_in={7: -1, 8: 0, 10: -2})
            pedes.update(param._SIMULATION_TIME)
            self.addVehicle(pedes)
            # static object
            obs1 = StaticObject(
                idx=1,
                poly=np.array([[-20, -10], [-20, -8], [-15, -3],
                               [-5, -3], [-5, -15], [-15, -15]]))
            self.addStaticObject(obs1)

            obs2 = StaticObject(
                idx=2,
                poly=np.array([[0, -10], [0, -4], [10, -4], [8, -10]]))
            self.addStaticObject(obs2)

            obs3 = StaticObject(
                idx=3,
                poly=np.array([[5, 3], [10, 3], [10, 7], [5, 7]]))
            # self.addStaticObject(obs3)

            # road boundary
            road1 = RoadBoundary(scenario=3)
            self.addRoadBoundary(road1)

        if scenario == 4:
            # pedestrian
            pedes = OtherVehicle(idx=3, length=1, width=1)
            startPose3 = Pose(
                x_m=-3, y_m=-7, yaw_rad=np.pi/2,
                covLatLong=np.array([[1, 0.0], [0.0, 0.5]]),
                vdy=VehicleDynamic(4, 0), timestamp_s=5.4)
            pedes.start(startPose=startPose3, u_in=0)
            pedes.predict(l_u_in={7: -1, 8: 0, 10: -2})
            pedes.update(param._SIMULATION_TIME)
            # self.addVehicle(pedes)
            # static object
            obs1 = StaticObject(
                idx=1,
                poly=np.array([[-15, -5], [-15, -2.5], [-10, -3],
                               [0, -2.5], [0, -10]]))
            self.addStaticObject(obs1)

            obs2 = StaticObject(
                idx=2,
                poly=np.array([[0, -10], [0, -4], [10, -4], [8, -10]]))
            # self.addStaticObject(obs2)

            obs3 = StaticObject(
                idx=3,
                poly=np.array([[5, 3], [10, 3], [10, 7], [5, 7]]))
            # self.addStaticObject(obs3)

            # road boundary
            road1 = RoadBoundary(scenario=2)
            self.addRoadBoundary(road1)
        
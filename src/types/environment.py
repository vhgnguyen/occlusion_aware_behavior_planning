import matplotlib.pyplot as plt
import numpy as np
import math

import pose_functions as pfnc
from pose import Pose, VehicleDynamic
from objects import (StaticObject, RoadBoundary,
                     PedestrianCross, Vehicle, Pedestrian)
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
        self._l_hypoCoord = []

    def countPedestrian(self):
        return len(self._l_pedestrian)

    def countVehicle(self):
        return len(self._l_vehicle)

    def countStaticObject(self):
        return len(self._l_staticObject)

    def addRoadBoundary(self, roadBoundary: RoadBoundary):
        self._l_road.append(roadBoundary)

    def addVehicle(self, otherVehicle: Vehicle):
        self._l_vehicle.append(otherVehicle)

    def addStaticObject(self, staticObject: StaticObject):
        self._l_staticObject.append(staticObject)

    def addPedestrian(self, pedestrian: Pedestrian):
        self._l_pedestrian.append(pedestrian)

    def addPedestrianCross(self, cross: PedestrianCross):
        self._l_cross.append(cross)

    def move(self, currentTime: float, dT=param._dT):
        """
        Move objects in environment to next step
        """
        for vehicle in self._l_vehicle:
            if currentTime >= vehicle._startTime:
                vehicle.move()
        for pedestrian in self._l_pedestrian:
            if currentTime >= pedestrian._startTime:
                pedestrian.move()

    def updateAt(self, x_m: float, y_m: float, timestamp_s,
                 radius=param._SCAN_RADIUS):
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

    def update(self, pose: Pose, from_timestamp: float,
               radius=param._SCAN_RADIUS):
        """
        Get environment around a given position
        """
        self._l_hypoCoord = []
        l_staticVehicle = []
        l_vehicle = []
        l_object = []
        l_pedestrian = []
        l_polys = []
        currentPos = np.array([pose.x_m, pose.y_m])

        # find static object
        for sObj in self._l_staticObject:
            objPoly = sObj._poly
            d2ego = np.linalg.norm(objPoly - currentPos, axis=1)
            if ((d2ego) < radius).any():
                l_object.append(sObj)
                l_polys.append(objPoly)
                self.generateHypothesis(pose, sObj)

        # find static vehicle
        for veh in self._l_vehicle:
            vehPose = veh.getCurrentPose()
            if vehPose.timestamp_s == from_timestamp:
                vehPos = np.array([vehPose.x_m, vehPose.y_m])
                if np.linalg.norm(vehPos - currentPos) < radius:
                    if vehPose.vdy.vx_ms < 2:
                        l_polys.append(veh.getPoly(from_timestamp))
                        l_staticVehicle.append(veh)

        # generate field of view
        fov = pfnc.FOV(
            pose=pose, polys=l_polys, angle=param._FOV_ANGLE,
            radius=radius, nrRays=param._FOV_RAYS)

        # check other moving vehicle in FOV
        for veh in self._l_vehicle:
            vehPose = veh.getCurrentPose()
            if vehPose.timestamp_s == from_timestamp:
                vehPos = np.array([vehPose.x_m, vehPose.y_m])
                if np.linalg.norm(vehPos - currentPos) < radius:
                    vehPoly = veh.getCurrentPoly()
                    if pfnc.inPolygon(vehPoly, fov):
                        veh.setDetected(True)
                        l_vehicle.append(veh)
                        continue
            veh.setDetected(False)

        # check pedestrian in FOV
        for pedes in self._l_pedestrian:
            pedesPose = pedes.getCurrentPose()
            if pedesPose.timestamp_s == from_timestamp:
                pedesPos = np.array([pedesPose.x_m, pedesPose.y_m])
                if np.linalg.norm(pedesPos - currentPos) < radius:
                    if pfnc.inPolygonPoint(pedesPos, fov):
                        pedes.setDetected(True)
                        l_pedestrian.append(pedes)
                        continue
            pedes.setDetected(False)

        l_update = {'vehicle': l_vehicle,
                    'staticObject': l_object,
                    'pedestrian': l_pedestrian}

        return l_update, fov

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
                poly=np.array([[-40, 8], [-20, 8], [-10, 15],
                              [-10, 20], [-40, 20]]))
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

    def generateHypothesis(self, pose, obj, dThres=1, radius=param._SCAN_RADIUS+5):
        randVertex, alpha = pfnc.minFOVAngle(pose, poly=obj._poly)

        if alpha is None:
            return
        l1_1 = np.array([pose.x_m, pose.y_m])

        dS = np.sqrt((randVertex[0]-pose.x_m)**2 + (randVertex[1]-pose.y_m)**2)
        d2MP = dS * math.cos(alpha)
        MP = pose.heading() * d2MP + l1_1

        p2r = randVertex - l1_1
        p2r /= p2r / np.linalg.norm(p2r)
        l1_2 = p2r * radius + l1_1

        # find intersection with pedestrian cross
        for c in self._l_cross:
            ip_l = pfnc.seg_intersect(l1_1, l1_2, c.left[0], c.left[1])
            ip_r = pfnc.seg_intersect(l1_1, l1_2, c.right[0], c.right[1])
            # if ip_l is not None:
            self._l_hypoCoord.append([ip_l, ip_r, l1_2])


        # heading = np.array([math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)])
        # dS = np.sqrt((randVertex[0]-pose.x_m)**2 + (randVertex[1]-pose.y_m)**2)
        # dToMergePoint = dS * math.cos(alpha) + dThres
        # MP = heading * dToMergePoint + np.array([pose.x_m, pose.y_m])
        # visibleDistance = dToMergePoint * np.tan(alpha)

        
# ---------------------- BACK UP FUNCTIONS -----------------------------
        
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
        
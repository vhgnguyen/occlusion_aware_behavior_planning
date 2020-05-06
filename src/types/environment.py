import matplotlib.pyplot as plt
import numpy as np
import math
from shapely.geometry import Polygon

import pose_functions as pfnc
from pose import Pose, VehicleDynamic
from objects import (StaticObject, RoadBoundary,
                     PedestrianCross, Vehicle, Pedestrian)
import _param as param

import OpenGL.GL as gl

class Environment(object):
    """
        Class define environment
    """
    def __init__(self):
        self._l_road = []
        self._l_staticObject = []
        self._l_vehicle = []
        self._l_pedestrian = []
        self._l_cross = []
        self._l_hypoPedes = []
        self._l_hypoVehicle = []

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

    def updateAt(self, pose, from_timestamp_s, u_in,
                 radius=param._SCAN_RADIUS):
        """
        Get environment around a given position
        """
        self._l_hypoPedes = []
        self._l_hypoVehicle = []

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
                self.generateHypothesis(pose, objPoly, u_in)

        # find vehicle
        for veh in self._l_vehicle:
            vehPose = veh.getPoseAt(from_timestamp_s)
            if vehPose is not None and vehPose.timestamp_s == from_timestamp_s:
                vehPos = np.array([vehPose.x_m, vehPose.y_m])
                if np.linalg.norm(vehPos - currentPos) < radius:
                    vehPoly = veh.getPoly(from_timestamp_s)
                    l_polys.append(vehPoly)
                    # self.generateHypothesis(pose, vehPoly)

        # generate field of view
        fov = pfnc.FOV(
            pose=pose, polys=l_polys, angle=param._FOV_ANGLE,
            radius=radius, nrRays=param._FOV_RAYS)
        fov_poly = Polygon(fov)

        # check other moving vehicle in FOV
        for veh in self._l_vehicle:
            vehPose = veh.getPoseAt(from_timestamp_s)
            if vehPose is not None and vehPose.timestamp_s == from_timestamp_s:
                vehPos = np.array([vehPose.x_m, vehPose.y_m])
                if np.linalg.norm(vehPos - currentPos) < radius:
                    vehPoly = veh.getCurrentPoly()
                    if pfnc.inPolyPointList(vehPoly, fov_poly):
                        veh.setDetected(True)
                        # if vehPose.vdy.vx_ms > 0:
                        l_vehicle.append(veh)
                        continue
                    else:
                        veh.setDetected(False)
                        continue
            veh.setDetected(None)

        # check pedestrian in FOV
        for pedes in self._l_pedestrian:
            pedesPose = pedes.getPoseAt(from_timestamp_s)
            if pedesPose is not None and pedesPose.timestamp_s == from_timestamp_s:
                pedesPos = np.array([pedesPose.x_m, pedesPose.y_m])
                if np.linalg.norm(pedesPos - currentPos) < radius:
                    # if pfnc.inPolygonPoint(pedesPos, fov):
                    if pfnc.inPolyPoint(pedesPos, fov_poly):
                        pedes.setDetected(True)
                        l_pedestrian.append(pedes)
                        continue
                    else:
                        pedes.setDetected(False)
                        continue
            pedes.setDetected(None)

        l_update = {'vehicle': l_vehicle,
                    'staticObject': l_object,
                    'pedestrian': l_pedestrian,
                    'hypoPedestrian': self._l_hypoPedes,
                    'hypoVehicle': self._l_hypoVehicle}

        return l_update

    def update(self, pose: Pose, from_timestamp: float, u_in,
               radius=param._SCAN_RADIUS):
        """
        Get environment around a given position
        """
        self._l_hypoPedes = []
        self._l_hypoVehicle = []

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
                self.generateHypothesis(pose, objPoly, u_in)

        # find vehicle
        for veh in self._l_vehicle:
            vehPose = veh.getCurrentPose()
            if vehPose.timestamp_s == from_timestamp:
                vehPos = np.array([vehPose.x_m, vehPose.y_m])
                if np.linalg.norm(vehPos - currentPos) < radius:
                    vehPoly = veh.getPoly(from_timestamp)
                    l_polys.append(vehPoly)
                    if vehPose.vdy.vx_ms == 0:
                        self.generateHypothesis(pose, vehPoly, u_in)

        # generate field of view
        fov = pfnc.FOV(
            pose=pose, polys=l_polys, angle=param._FOV_ANGLE,
            radius=radius, nrRays=param._FOV_RAYS)
        fov_poly = Polygon(fov)

        # check other moving vehicle in FOV
        for veh in self._l_vehicle:
            vehPose = veh.getCurrentPose()
            if vehPose.timestamp_s == from_timestamp:
                vehPos = np.array([vehPose.x_m, vehPose.y_m])
                if np.linalg.norm(vehPos - currentPos) < radius:
                    vehPoly = veh.getCurrentPoly()
                    # if pfnc.inPolygon(vehPoly, fov):
                    if pfnc.inPolyPointList(vehPoly, fov_poly):
                        veh.setDetected(True)
                        if vehPose.vdy.vx_ms > 0:
                            l_vehicle.append(veh)
                        continue
                    else:
                        veh.setDetected(False)
                        continue
            veh.setDetected(False)

        # check pedestrian in FOV
        for pedes in self._l_pedestrian:
            pedesPose = pedes.getCurrentPose()
            if pedesPose.timestamp_s == from_timestamp:
                pedesPos = np.array([pedesPose.x_m, pedesPose.y_m])
                if np.linalg.norm(pedesPos - currentPos) < radius:
                    # if pfnc.inPolygonPoint(pedesPos, fov):
                    if pfnc.inPolyPoint(pedesPos, fov_poly):
                        pedes.setDetected(True)
                        l_pedestrian.append(pedes)
                        continue
                    else:
                        pedes.setDetected(False)
                        continue
            pedes.setDetected(False)

        l_update = {'vehicle': l_vehicle,
                    'staticObject': l_object,
                    'pedestrian': l_pedestrian,
                    'hypoPedestrian': self._l_hypoPedes,
                    'hypoVehicle': self._l_hypoVehicle}

        return l_update, fov

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
                poly=np.array([[-50, -20], [-39, -20], [-39, -6],
                               [-45, -5], [-50, -5]]))
            self.addStaticObject(obs1)

            obs2 = StaticObject(
                idx=2,
                poly=np.array([[-35, -20], [-20, -20], [-20, -5],
                               [-30, -5], [-35, -6]]))
            self.addStaticObject(obs2)

            obs3 = StaticObject(
                idx=3,
                poly=np.array([[-15, -20], [-5, -20], [-5, -10],
                               [-10, -6], [-15, -6]]))
            self.addStaticObject(obs3)

            obs4 = StaticObject(
                idx=4,
                poly=np.array([[-2, -20], [5, -20], [5, -10],
                               [4, -6], [-2, -6]]))
            self.addStaticObject(obs4)

            obs5 = StaticObject(
                idx=5,
                poly=np.array([[-60, 7], [0, 7], [6, 10],
                               [6, 20], [-60, 20]]))
            self.addStaticObject(obs5)

            # pedestrian cross
            cross1 = PedestrianCross(
                left=np.array([[5, -7], [5, 7]]),
                right=np.array([[7, -7], [7, 7]]),
                density=0.5
            )
            self.addPedestrianCross(cross1)

        elif scenario == 2:
            # static object
            obs1 = StaticObject(
                idx=1,
                poly=np.array([[-40, -20], [-8, -20], [-8, -6],
                               [-20, -5], [-40, -5]]))
            self.addStaticObject(obs1)

            obs2 = StaticObject(
                idx=2,
                poly=np.array([[8, -20], [30, -20], [30, -5], [8, -5]]))
            self.addStaticObject(obs2)

            # pedestrian cross
            cross1 = PedestrianCross(
                left=np.array([[-7, -10], [-7, 5]]),
                right=np.array([[-4, -10], [-4, 5]]),
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
                poly=np.array([[-40, 5], [-20, 5], [-10, 8],
                              [-10, 20], [-40, 20]]))
            self.addStaticObject(obs3)

            obs4 = StaticObject(
                idx=4,
                poly=np.array([[5, 5], [10, 5], [10, 20],
                              [5, 20]]))
            self.addStaticObject(obs4)

            # pedestrian cross
            cross1 = PedestrianCross(
                left=np.array([[-7, -10], [-7, 10]]),
                right=np.array([[-4, -10], [-4, 10]]),
                density=0.8
            )
            self.addPedestrianCross(cross1)
        
        elif scenario == 4:
            # static object
            obs1 = StaticObject(
                idx=1,
                poly=np.array([[-40, -20], [-6, -20], [-6, -8],
                               [-20, -6], [-40, -6]]))
            self.addStaticObject(obs1)

            obs2 = StaticObject(
                idx=2,
                poly=np.array([[8, -20], [30, -20], [30, -5], [8, -5]]))
            self.addStaticObject(obs2)

            obs3 = StaticObject(
                idx=3,
                poly=np.array([[-40, 5], [-20, 5], [-10, 8],
                              [-10, 20], [-40, 20]]))
            self.addStaticObject(obs3)

            obs4 = StaticObject(
                idx=4,
                poly=np.array([[5, 5], [10, 5], [10, 20],
                              [5, 20]]))
            self.addStaticObject(obs4)

            # pedestrian cross
            cross1 = PedestrianCross(
                left=np.array([[-6, -10], [-6, 10]]),
                right=np.array([[-4, -10], [-4, 10]]),
                density=0.8
            )
            cross2 = PedestrianCross(
                left=np.array([[-4, 8], [-4, 8]]),
                right=np.array([[-4, 6], [-4, 6]]),
                density=0.8
            )
            self.addPedestrianCross(cross1)
            self.addPedestrianCross(cross2)

        # road boundary
        road = RoadBoundary(scenario=scenario)
        self._l_road = road.l_road

    def generateHypothesis(self, pose, objPoly, u_in, dThres=2, radius=param._SCAN_RADIUS):
        randVertex, alpha = pfnc.minFOVAngle(pose, poly=objPoly)
        if alpha is None:
            return

        l1_1 = np.array([pose.x_m, pose.y_m])
        p2r = randVertex - l1_1
        p2r_l2 = np.linalg.norm(p2r)
        p2r_norm = p2r / p2r_l2
        l1_2 = p2r_norm * radius + l1_1
        ca = math.cos(alpha)
        crossPedes = False
        crossRoad = False

        # find intersection with pedestrian cross
        for c in self._l_cross:
            ip_l = pfnc.seg_intersect(l1_1, l1_2, c.left[0], c.left[1])
            ip_r = pfnc.seg_intersect(l1_1, l1_2, c.right[0], c.right[1])
            d2MP_l, d2MP_r = 9999, 9999
            if ip_l is not None:
                l1_ip_l = np.linalg.norm(ip_l-l1_1)
                if l1_ip_l < p2r_l2:
                    continue
                d2MP_l = (l1_ip_l + dThres) * ca
                crossPedes = True

            if ip_r is not None:
                l1_ip_r = np.linalg.norm(ip_r-l1_1)
                if l1_ip_r < p2r_l2:
                    continue
                d2MP_r = (l1_ip_r + dThres) * ca
                crossPedes = True

            if d2MP_l < d2MP_r:
                MP_l = d2MP_l * pose.heading() + l1_1
                ip_l += p2r_norm * dThres
                heading = MP_l - ip_l
                heading /= np.linalg.norm(heading)
                ip_l -= heading * dThres
                hypoPedes = Pedestrian(
                    idx=99, from_x_m=ip_l[0], from_y_m=ip_l[1],
                    to_x_m=MP_l[0], to_y_m=MP_l[1], covLong=0.5, covLat=0.5,
                    vx_ms=param._PEDESTRIAN_VX, startTime=pose.timestamp_s,
                    appearRate=0.8)
                if abs(abs(hypoPedes.theta) - abs(c.theta)) < np.pi/3:
                    self._l_hypoPedes.append(hypoPedes)

            elif d2MP_l > d2MP_r:
                MP_r = d2MP_r * pose.heading() + l1_1
                ip_r += p2r_norm * dThres
                heading = MP_r - ip_r
                heading /= np.linalg.norm(heading)
                ip_r -= heading * dThres
                hypoPedes = Pedestrian(
                    idx=99, from_x_m=ip_r[0], from_y_m=ip_r[1],
                    to_x_m=MP_r[0], to_y_m=MP_r[1], covLong=0.5, covLat=0.5,
                    vx_ms=param._PEDESTRIAN_VX, startTime=pose.timestamp_s,
                    appearRate=0.8)
                if abs(abs(hypoPedes.theta) - abs(c.theta)) < np.pi/3:
                    self._l_hypoPedes.append(hypoPedes)
            
        for road in self._l_road:
            lane_heading = np.array([np.cos(road.theta), np.sin(road.theta)])
            if road.left is not None:
                ip_l = pfnc.seg_intersect(l1_1, l1_2, road.left[0], road.left[1])
                if ip_l is not None:
                    l1_ip_l = np.linalg.norm(l1_1 - ip_l)
                    if l1_ip_l < p2r_l2:
                        continue

            if road.lane is not None:
                ip_m = pfnc.seg_intersect(l1_1, l1_2, road.lane[0], road.lane[1])
                if ip_m is not None:
                    l1_ip_m = np.linalg.norm(l1_1 - ip_m)
                    if l1_ip_m < p2r_l2:
                        continue

            if road.right is not None:
                ip_r = pfnc.seg_intersect(l1_1, l1_2, road.right[0], road.right[1])
                if ip_r is not None:
                    l1_ip_r = np.linalg.norm(l1_1 - ip_r)
                    if l1_ip_r < p2r_l2:
                        continue

            if ip_l is not None and ip_m is not None:
                endPos = (ip_l + ip_m) / 2
                startPos = endPos + lane_heading * 2 * dThres
                if np.linalg.norm(endPos-l1_1) < np.linalg.norm(startPos-l1_1):
                    hypoVeh = Vehicle(
                        idx=99, length=param._CAR_LENGTH, width=param._CAR_WIDTH,
                        from_x_m=startPos[0], from_y_m=startPos[1],
                        to_x_m=endPos[0], to_y_m=endPos[1], covLong=1, covLat=0.5,
                        vx_ms=param._VEHICLE_VX, startTime=pose.timestamp_s,
                        appearRate=0.5)
                    self._l_hypoVehicle.append(hypoVeh)
                    crossRoad = True

            if ip_r is not None and ip_m is not None:
                endPos = (ip_r + ip_m) / 2
                startPos = endPos - lane_heading * 2 * dThres
                if np.linalg.norm(endPos-l1_1) < np.linalg.norm(startPos-l1_1):
                    hypoVeh = Vehicle(
                        idx=99, length=param._CAR_LENGTH, width=param._CAR_WIDTH,
                        from_x_m=startPos[0], from_y_m=startPos[1],
                        to_x_m=endPos[0], to_y_m=endPos[1], covLong=1, covLat=0.5,
                        vx_ms=param._VEHICLE_VX, startTime=pose.timestamp_s,
                        appearRate=0.5)
                    self._l_hypoVehicle.append(hypoVeh)
                    crossRoad = True

        if not crossPedes and not crossRoad:
        # if not crossPedes:
            dS = np.sqrt((randVertex[0]-pose.x_m)**2 + (randVertex[1]-pose.y_m)**2)
            d2MP = (dS + dThres) * ca
            MP = pose.heading() * d2MP + l1_1
            startPos = randVertex + p2r_norm * dThres
            heading = MP - startPos
            heading /= np.linalg.norm(heading)
            startPos -= heading * dThres * 0.5
            # if not pfnc.inPolygonPoint(startPos, objPoly):
            hypoPedes = Pedestrian(
                    idx=99, from_x_m=startPos[0], from_y_m=startPos[1],
                    to_x_m=MP[0], to_y_m=MP[1], covLong=0.5, covLat=0.5,
                    vx_ms=param._PEDESTRIAN_VX, startTime=pose.timestamp_s,
                    appearRate=0.2)
            self._l_hypoPedes.append(hypoPedes)

    def restart(self):
        for veh in self._l_vehicle:
            veh.restart()
        for pedes in self._l_pedestrian:
            pedes.restart()

# ---------------------- BACK UP FUNCTIONS -----------------------------

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

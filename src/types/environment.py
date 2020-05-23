import matplotlib.pyplot as plt
import numpy as np
import math
from shapely.geometry import Polygon

import pose_functions as pfnc
import risk_functions as rfnc
import set_scenario as sc
from pose import Pose, VehicleDynamic
from objects import (StaticObject, Road, PedestrianCross,
                     Vehicle, Pedestrian)
import _param as param


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

    def restart(self):
        for veh in self._l_vehicle:
            veh.restart()
        for pedes in self._l_pedestrian:
            pedes.restart()

    def setScenario(self, scenario):
        self._l_staticObject, self._l_cross, self._l_road = sc.setScenario(scenario)

    def countPedestrian(self):
        return len(self._l_pedestrian)

    def countVehicle(self):
        return len(self._l_vehicle)

    def countStaticObject(self):
        return len(self._l_staticObject)

    def addRoad(self, road: Road):
        self._l_road.append(road)

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

    def update(self, pose: Pose, from_timestamp: float, u_in,
               radius, hypothesis=False):
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

        # find vehicle
        for veh in self._l_vehicle:
            vehPose = veh.getCurrentPose()
            if vehPose.timestamp_s == from_timestamp:
                vehPos = np.array([vehPose.x_m, vehPose.y_m])
                if np.linalg.norm(vehPos - currentPos) < radius:
                    vehPoly = veh.getCurrentPoly()
                    l_polys.append(vehPoly)

        # generate field of view
        fov = pfnc.FOV(
            pose=pose, polys=l_polys, angle=param._FOV_ANGLE,
            radius=radius, nrRays=param._FOV_RAYS)
        fov_poly = Polygon(fov)

        # generate hypothesis from static object
        for sObj in self._l_staticObject:
            objPoly = sObj._poly
            if pfnc.inPolyPointList(objPoly, fov_poly) and  hypothesis:
                self._generateHypothesis(pose, objPoly, fov_poly, u_in, radius=radius)

        # check other moving vehicle in FOV
        for veh in self._l_vehicle:
            vehPose = veh.getCurrentPose()
            if vehPose.timestamp_s == from_timestamp:
                vehPos = np.array([vehPose.x_m, vehPose.y_m])
                if np.linalg.norm(vehPos - currentPos) < radius:
                    vehPoly = veh.getCurrentPoly()
                    if hypothesis and vehPose.vdy.vx_ms == 0:
                        self._generateHypothesis(
                            pose, vehPoly, fov_poly, u_in, radius=radius,
                            objectVehicle=True)
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
                        if pedesPose.vdy.vx_ms > 0:
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

    def _generateHypothesis(self, pose, objPoly, fov_poly, u_in, radius, objectVehicle=False):

        randVertex, alpha = pfnc.minFOVAngle(pose, poly=objPoly)
        if alpha is None:
            return

        crossPedes = False
        crossRoad = False
        dThres = 2  # define short distance from boundary lines:D

        # get occlusion boundary
        ca = math.cos(alpha)
        dS = np.sqrt((randVertex[0]-pose.x_m)**2 + (randVertex[1]-pose.y_m)**2)
        d2MP = (dS + dThres) * ca
        MP = pose.heading() * d2MP + np.array([pose.x_m, pose.y_m])
        interactRate = rfnc.interactRate(d2MP)

        # compute distance to ostacle bound vertex
        l1_1 = np.array([pose.x_m, pose.y_m])
        p2r = randVertex - l1_1
        p2r_l2 = np.linalg.norm(p2r)
        p2r_norm = p2r / p2r_l2
        l1_2 = p2r_norm * radius + l1_1

        # find intersection with pedestrian cross
        for c in self._l_cross:
            ip_l = pfnc.seg_intersect(l1_1, l1_2, c.left[0], c.left[1])
            ip_r = pfnc.seg_intersect(l1_1, l1_2, c.right[0], c.right[1])
            d2MP_l, d2MP_r = 9999, 9999

            # find direction of potential pedestrian
            if ip_l is not None:
                l1_ip_l = np.linalg.norm(ip_l-l1_1)
                if l1_ip_l < p2r_l2:
                    continue
                d2MP_l = (l1_ip_l + dThres) * ca
            if ip_r is not None:
                l1_ip_r = np.linalg.norm(ip_r-l1_1)
                if l1_ip_r < p2r_l2:
                    continue
                d2MP_r = (l1_ip_r + dThres) * ca
            if d2MP_l < d2MP_r:
                MP_l = d2MP_l * pose.heading() + l1_1
                ip_l += p2r_norm * dThres
                heading = MP_l - ip_l
                heading /= np.linalg.norm(heading)
                ip_l -= heading * dThres
                hypoPedes = Pedestrian(
                    idx=99, from_x_m=ip_l[0], from_y_m=ip_l[1],
                    to_x_m=MP_l[0], to_y_m=MP_l[1],
                    covLong=param._HYPOPEDES_COV_LON,
                    covLat=param._HYPOPEDES_COV_LAT,
                    vx_ms=param._HYPOPEDES_VX, startTime=pose.timestamp_s,
                    appearRate=param._PEDES_APPEAR_RATE_CROSS)
                crossPedes = True
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
                    to_x_m=MP_r[0], to_y_m=MP_r[1],
                    covLong=param._HYPOPEDES_COV_LON,
                    covLat=param._HYPOPEDES_COV_LAT,
                    vx_ms=param._HYPOPEDES_VX, startTime=pose.timestamp_s,
                    appearRate=param._PEDES_APPEAR_RATE_CROSS)
                crossPedes = True
                if abs(abs(hypoPedes.theta) - abs(c.theta)) < np.pi/3:
                    self._l_hypoPedes.append(hypoPedes)

        # find intersection with roads
        for road in self._l_road:
            lane_heading = np.array([np.cos(road.theta), np.sin(road.theta)])
            if road.left is not None:
                ip_l = pfnc.seg_intersect(l1_1, l1_2, road.left[0], road.left[1])
                if ip_l is not None:
                    l1_ip_l = np.linalg.norm(l1_1 - ip_l)

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

            if ip_l is not None and ip_m is not None:
                endPos = (ip_l + ip_m) / 2
                startPos = endPos + lane_heading * 2 * dThres
                if not pfnc.inPolyPoint(startPos, fov_poly):
                    hypoVeh = Vehicle(
                        idx=99, length=param._CAR_LENGTH, width=param._CAR_WIDTH,
                        from_x_m=startPos[0], from_y_m=startPos[1],
                        to_x_m=endPos[0], to_y_m=endPos[1],
                        covLong=param._HYPOVEH_COV_LON,
                        covLat=param._HYPOVEH_COV_LAT,
                        vx_ms=param._HYPOVEH_VX, startTime=pose.timestamp_s,
                        appearRate=param._APPEAR_RATE_VEH,
                        interactRate=interactRate)
                    self._l_hypoVehicle.append(hypoVeh)
                    crossRoad = True

            if ip_r is not None and ip_m is not None:
                endPos = (ip_r + ip_m) / 2
                startPos = endPos - lane_heading * 2 * dThres
                if not pfnc.inPolyPoint(startPos, fov_poly):
                    hypoVeh = Vehicle(
                        idx=99, length=param._CAR_LENGTH, width=param._CAR_WIDTH,
                        from_x_m=startPos[0], from_y_m=startPos[1],
                        to_x_m=endPos[0], to_y_m=endPos[1],
                        covLong=param._HYPOVEH_COV_LON,
                        covLat=param._HYPOVEH_COV_LAT,
                        vx_ms=param._HYPOVEH_VX, startTime=pose.timestamp_s,
                        appearRate=param._APPEAR_RATE_VEH,
                        interactRate=interactRate)
                    self._l_hypoVehicle.append(hypoVeh)
                    crossRoad = True

        # if not crossPedes and not crossRoad:
        startPos = randVertex + p2r_norm * dThres
        d = np.linalg.norm(startPos - MP)
        if d < param._PEDES_OTHER_MIN_THRESHOLD:
            heading = MP - startPos
            heading /= np.linalg.norm(heading)
            startPos -= heading
            if not pfnc.inPolyPoint(startPos, fov_poly):
                appearRate = param._PEDES_APPEAR_RATE_STREET \
                    if objectVehicle else param._PEDES_APPEAR_RATE_OTHER
                hypoPedes = Pedestrian(
                    idx=99, from_x_m=startPos[0], from_y_m=startPos[1],
                    to_x_m=MP[0], to_y_m=MP[1],
                    covLong=param._HYPOPEDES_COV_LON,
                    covLat=param._HYPOPEDES_COV_LAT,
                    vx_ms=param._HYPOPEDES_VX + param._HYPOPEDES_OFFSET_VX,
                    startTime=pose.timestamp_s,
                    appearRate=param._PEDES_APPEAR_RATE_OTHER)
                self._l_hypoPedes.append(hypoPedes)

    def updateAt(self, pose, from_timestamp, u_in,
                 radius, hypothesis):
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

        # find vehicle
        for veh in self._l_vehicle:
            vehPose = veh.getCurrentPose()
            if vehPose.timestamp_s == from_timestamp:
                vehPos = np.array([vehPose.x_m, vehPose.y_m])
                if np.linalg.norm(vehPos - currentPos) < radius:
                    vehPoly = veh.getPoly(from_timestamp)
                    l_polys.append(vehPoly)

        # generate field of view
        fov = pfnc.FOV(
            pose=pose, polys=l_polys, angle=param._FOV_ANGLE,
            radius=radius, nrRays=param._FOV_RAYS)
        fov_poly = Polygon(fov)

        # find static object
        for sObj in self._l_staticObject:
            objPoly = sObj._poly
            if pfnc.inPolyPointList(objPoly, fov_poly) and hypothesis:
                self._generateHypothesis(pose, objPoly, fov_poly, u_in, radius)

        # check other moving vehicle in FOV
        for veh in self._l_vehicle:
            vehPose = veh.getPoseAt(from_timestamp)
            if vehPose is None:
                continue
            vehPos = np.array([vehPose.x_m, vehPose.y_m])
            if np.linalg.norm(vehPos - currentPos) < radius:
                vehPoly = veh.getPoly(from_timestamp)
                if hypothesis and vehPose.vdy.vx_ms == 0:
                    self._generateHypothesis(pose, vehPoly, fov_poly, u_in, radius)
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
            pedesPose = pedes.getPoseAt(from_timestamp)
            if pedesPose is None:
                continue
            pedesPos = np.array([pedesPose.x_m, pedesPose.y_m])
            if np.linalg.norm(pedesPos - currentPos) < radius:
                if pfnc.inPolyPoint(pedesPos, fov_poly):
                    pedes.setDetected(True)
                    if pedesPose.vdy.vx_ms > 0:
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

        return l_update

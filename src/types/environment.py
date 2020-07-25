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
        # elements
        self._l_road = []
        self._l_staticObject = []
        self._l_vehicle = []
        self._l_pedestrian = []
        self._l_cross = []

        # hypothesis
        self._l_hypoPedes = []
        self._l_hypoVehicle = []
        
        # output to ego car
        self._l_update = {}
        self._fov = []
        self._fovRange = 0

        self._isStarted = False

    def isStarted(self):
        return self._isStarted

    def restart(self):
        for veh in self._l_vehicle:
            veh.restart()
        for pedes in self._l_pedestrian:
            pedes.restart()
        self._l_update = {}
        self._isStarted = False

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

    def getFOV(self):
        return self._fov, self._fovRange

    def getCurrentObjectList(self):
        return self._l_update

    def move(self, egoPose, currentTime: float, dT=param._dT):
        """
        Move objects in environment to next step
        """
        for vehicle in self._l_vehicle:
            if currentTime >= vehicle._startTime:
                vehicle.move(dT=dT)
        for pedestrian in self._l_pedestrian:
            if currentTime >= pedestrian._startTime:
                pedestrian.move(dT=dT)
        self.update(egoPose=egoPose)

    def update(self, egoPose):
        self._update(
            pose=egoPose,
            from_timestamp=egoPose.timestamp_s,
            radius=param._SCAN_RADIUS,
            hypothesis=param._ENABLE_HYPOTHESIS,
            pVx=param._HYPOPEDES_VX,
            pCovLat=param._HYPOPEDES_COV_LAT,
            pCovLon=param._HYPOPEDES_COV_LON,
            pCrossRate=param._PEDES_APPEAR_RATE_CROSS,
            pStreetRate=param._PEDES_APPEAR_RATE_STREET,
            pOtherRate=param._PEDES_APPEAR_RATE_OTHER,
            pDistanceThres=param._PEDES_OTHER_MIN_THRESHOLD,
            vVx=param._HYPOVEH_VX,
            vCovLat=param._HYPOVEH_COV_LAT,
            vCovLon=param._HYPOVEH_COV_LON,
            vRate=param._APPEAR_RATE_VEH
            )
        if not self._isStarted:
            self._isStarted = True

    def _update(self, pose: Pose, from_timestamp: float,
               radius, hypothesis=False,
               pVx=param._HYPOPEDES_VX,
               pCovLat=param._HYPOPEDES_COV_LAT,
               pCovLon=param._HYPOPEDES_COV_LON,
               pCrossRate=param._PEDES_APPEAR_RATE_CROSS,
               pStreetRate=param._PEDES_APPEAR_RATE_STREET,
               pOtherRate=param._PEDES_APPEAR_RATE_OTHER,
               pDistanceThres=param._PEDES_OTHER_MIN_THRESHOLD,
               vVx=param._HYPOVEH_VX,
               vCovLat=param._HYPOVEH_COV_LAT,
               vCovLon=param._HYPOVEH_COV_LON,
               vRate=param._APPEAR_RATE_VEH):
        """
        Get environment around a given position
        """
        self._l_hypoPedes = []
        self._l_hypoVehicle = []
        l_fov = {}
        l_vehicle = []
        l_staticVehicle = []
        l_object = []
        l_pedestrian = []
        l_polys = []
        currentPos = np.array([pose.x_m, pose.y_m])
        searchRadius = radius + 1
        # find static object
        for sObj in self._l_staticObject:
            objPoly = sObj._poly
            d2ego = np.linalg.norm(objPoly - currentPos, axis=1)
            if ((d2ego) < radius).any():
                l_object.append(sObj)
                l_polys.append(objPoly)

        # find vehicle poly 
        for veh in self._l_vehicle:
            vehPose = veh.getCurrentPose()
            if vehPose.timestamp_s == from_timestamp:
                vehPos = np.array([vehPose.x_m, vehPose.y_m])
                if np.linalg.norm(vehPos - currentPos) < searchRadius:
                    vehPoly = veh.getCurrentPoly()
                    l_polys.append(vehPoly)

        # generate field of view
        self._fov, self._fovRange = pfnc.FOV(
            pose=pose, polys=l_polys, angle=param._FOV_ANGLE,
            radius=radius, nrRays=param._FOV_RAYS)
        fov_poly = Polygon(self._fov)

        # generate hypothesis from static object
        for sObj in l_object:
            objPoly = sObj._poly
            if pfnc.inPolyPointList(objPoly, fov_poly) and hypothesis:
                l_fov_d, hasHypo = self._generateHypothesis(
                    pose, objPoly, fov_poly, radius,
                    pVx, pCovLat, pCovLon, pCrossRate, pStreetRate, pOtherRate,
                    pDistanceThres, vVx, vCovLat, vCovLon, vRate)
                if hasHypo:
                    l_fov.update({sObj._idx: l_fov_d})

        # check other moving vehicle in FOV
        for veh in self._l_vehicle:
            vehPose = veh.getCurrentPose()
            if vehPose.timestamp_s == from_timestamp:
                vehPos = np.array([vehPose.x_m, vehPose.y_m])
                # if np.linalg.norm(vehPos - currentPos) < searchRadius:
                vehPoly = veh.getCurrentPoly()
                if pfnc.inPolyPointList(vehPoly, fov_poly):
                    veh.setDetected(True)
                    veh.setDetectedTime()
                    # generate hypothesis with low speed vehicle
                    if hypothesis and vehPose.vdy.vx_ms < 3:
                        l_fov_d, hasHypo = self._generateHypothesis(
                            pose, vehPoly, fov_poly, radius,
                            pVx, pCovLat, pCovLon, pCrossRate, pStreetRate, pOtherRate,
                            pDistanceThres, vVx, vCovLat, vCovLon, vRate,
                            objectVehicle=True)
                        if hasHypo:
                            l_fov.update({veh._idx: l_fov_d})
                    if vehPose.vdy.vx_ms > 0.5:
                        l_vehicle.append(veh)
                    else:
                        l_staticVehicle.append(veh)
                    continue
                else:
                    veh.setDetected(False)
                    continue
            # else:
            #     veh.setDetected(False)

        # check pedestrian in FOV
        for pedes in self._l_pedestrian:
            pedesPose = pedes.getCurrentPose()
            if pedesPose.timestamp_s == from_timestamp:
                pedesPos = np.array([pedesPose.x_m, pedesPose.y_m])
                if np.linalg.norm(pedesPos - currentPos) < searchRadius:
                    # if pfnc.inPolygonPoint(pedesPos, fov):
                    if pfnc.inPolyPoint(pedesPos, fov_poly):
                        pedes.setDetected(True)
                        pedes.setDetectedTime()
                        if pedesPose.vdy.vx_ms > 0:
                            l_pedestrian.append(pedes)
                        continue
                    else:
                        pedes.setDetected(False)
                        continue
            pedes.setDetected(False)

        self._l_update = {'vehicle': l_vehicle,
                          'staticObject': l_object,
                          'pedestrian': l_pedestrian,
                          'hypoPedestrian': self._l_hypoPedes,
                          'hypoVehicle': self._l_hypoVehicle,
                          'staticVehicle': l_staticVehicle,
                          'fovDistance': l_fov}
        del l_fov, l_vehicle, l_staticVehicle, l_pedestrian, l_object, l_polys, currentPos, searchRadius

    def _generateHypothesis(self, pose, objPoly, fov_poly, radius, pVx,
                            pCovLat, pCovLon, pCrossRate, pStreetRate, pOtherRate,
                            pDistanceThres, vVx, vCovLat, vCovLon, vRate,
                            objectVehicle=False):

        randVertex, alpha = pfnc.minFOVAngle(pose, poly=objPoly)
        if alpha is None:
            return None, False

        hasHypo = False
        crossPedes = False
        crossRoad = False
        dThres = 1  # define short distance from boundary lines:D

        # get occlusion boundary
        ca = math.cos(alpha)
        sa = math.sin(alpha)
        vx = pose.heading()
        l1_1 = np.array([pose.x_m, pose.y_m])
        l1_1 = l1_1 + pose.heading() * param._CAR_LENGTH * 0.5
        l_vx = l1_1 + vx * radius

        # compute end of occlusion point
        dS = np.sqrt((randVertex[0]-l1_1[0])**2 + (randVertex[1]-l1_1[1])**2)
        d2MP = (dS+dThres) * ca
        MP = l1_1 + d2MP * vx
        interactRate = rfnc.interactRate(d2MP, b=param._AWARENESS_DISTANCE)

        # compute distance to ostacle bound vertex
        p2r = randVertex - l1_1
        p2r_l2 = np.linalg.norm(p2r)
        p2r_norm = p2r / p2r_l2
        l1_2 = p2r_norm * radius + l1_1

        # export FOV
        D2MP_cross = 0
        D2MP_ll = 0
        D2MP_rl = 0
        fov_cross = 0  # crosswalk
        fov_str_l = 0  # left lane
        fov_str_r = 0  # right lane

        # find intersection with pedestrian cross
        for c in self._l_cross:
            # pVx = np.random.normal(pVx, 1, 1)[0]
            # pVx = max(pVx, 2)

            # noiseCovLon = np.random.normal(0, 3, 1)[0]
            # pCovLon = max(pCovLon, pCovLon+noiseCovLon)

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
                MP_l = d2MP_l * vx + l1_1
                ip_l += p2r_norm * dThres
                
                # FOV visibility
                fov_cross = np.linalg.norm(MP_l-ip_l)
                D2MP_cross = d2MP_l
                hasHypo = True

                heading = MP_l - ip_l
                heading /= np.linalg.norm(heading)
                ip_l -= heading * dThres
                hypoPedes = Pedestrian(
                    idx=99, from_x_m=ip_l[0], from_y_m=ip_l[1],
                    to_x_m=MP_l[0], to_y_m=MP_l[1],
                    covLong=pCovLon, covLat=pCovLat, vx_ms=pVx,
                    startTime=pose.timestamp_s, appearRate=pCrossRate,
                    interactRate=interactRate)
                if np.linalg.norm(ip_l - randVertex) < 5:
                    crossPedes = True
                if abs(abs(hypoPedes.theta) - abs(c.theta)) < np.pi/3:
                    self._l_hypoPedes.append(hypoPedes)

            elif d2MP_l > d2MP_r:
                MP_r = d2MP_r * vx + l1_1
                ip_r += p2r_norm * dThres

                # FOV visibility
                fov_cross = np.linalg.norm(MP_r-ip_r)
                D2MP_cross = d2MP_r
                hasHypo = True

                heading = MP_r - ip_r
                heading /= np.linalg.norm(heading)
                ip_r -= heading * dThres
                hypoPedes = Pedestrian(
                    idx=99, from_x_m=ip_r[0], from_y_m=ip_r[1],
                    to_x_m=MP_r[0], to_y_m=MP_r[1],
                    covLong=pCovLon, covLat=pCovLat, vx_ms=pVx,
                    startTime=pose.timestamp_s, appearRate=pCrossRate,
                    interactRate=interactRate)
                if np.linalg.norm(ip_r - randVertex) < 5:
                    crossPedes = True
                if abs(abs(hypoPedes.theta) - abs(c.theta)) < np.pi/3:
                    self._l_hypoPedes.append(hypoPedes)

        # find intersection with roads
        for road in self._l_road:
            # vVx = np.random.normal(vVx, 2, 1)[0]
            lane_heading = np.array([np.cos(road.theta), np.sin(road.theta)])
            lane_heading1 = lane_heading * radius
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
                # if not pfnc.inPolyPoint(startPos, fov_poly):
                # checkCross1 = pfnc.seg_intersect(l1_1, l_vx, startPos, startPos-lane_heading1)
                if pfnc.doIntersect(l1_1, l_vx, startPos, endPos-lane_heading1):
                    # FOV visibility
                    dE = np.linalg.norm(l1_1 - endPos)
                    D2MP_ll = dE * ca
                    fov_str_l = dE * sa
                    hasHypo = True

                    hypoVeh = Vehicle(
                        idx=99, length=param._CAR_LENGTH, width=param._CAR_WIDTH,
                        from_x_m=startPos[0], from_y_m=startPos[1],
                        to_x_m=endPos[0], to_y_m=endPos[1],
                        covLong=vCovLon, covLat=vCovLat, vx_ms=vVx,
                        startTime=pose.timestamp_s, appearRate=vRate,
                        interactRate=interactRate)
                    self._l_hypoVehicle.append(hypoVeh)
                    crossRoad = True

            if ip_r is not None and ip_m is not None:
                endPos = (ip_r + ip_m) / 2
                startPos = endPos - lane_heading * 2 * dThres
                # if not pfnc.inPolyPoint(startPos, fov_poly):
                # checkCross2 = pfnc.seg_intersect(l1_1, l_vx, startPos, startPos+lane_heading1)
                if pfnc.doIntersect(l1_1, l_vx, startPos, endPos+lane_heading1):
                    # FOV visibility
                    dE = np.linalg.norm(l1_1 - endPos)
                    D2MP_rl = dE * ca
                    fov_str_r = dE * sa
                    hasHypo = True

                    hypoVeh = Vehicle(
                        idx=99, length=param._CAR_LENGTH, width=param._CAR_WIDTH,
                        from_x_m=startPos[0], from_y_m=startPos[1],
                        to_x_m=endPos[0], to_y_m=endPos[1],
                        covLong=vCovLon, covLat=vCovLat, vx_ms=vVx,
                        startTime=pose.timestamp_s, appearRate=vRate,
                        interactRate=interactRate)
                    self._l_hypoVehicle.append(hypoVeh)
                    crossRoad = True

        # save FOV visibility
        l_fov_d = [fov_cross, fov_str_l, fov_str_r, D2MP_cross, D2MP_ll, D2MP_rl]

        # if crossPedes:
        #     return l_fov_d, hasHypo
        # startPos = randVertex + p2r_norm * dThres
        # d = np.linalg.norm(startPos - MP)
        # if d < pDistanceThres:
        #     heading = MP - startPos
        #     heading /= np.linalg.norm(heading)
        #     startPos -= heading
        #     if not pfnc.inPolyPoint(startPos, fov_poly):
        #         if objectVehicle:
        #             hypoPedes = Pedestrian(
        #                 idx=99, from_x_m=startPos[0], from_y_m=startPos[1],
        #                 to_x_m=MP[0], to_y_m=MP[1],
        #                 covLong=pCovLon, covLat=pCovLat, vx_ms=pVx,
        #                 startTime=pose.timestamp_s, appearRate=pStreetRate,
        #                 interactRate=interactRate)
        #             self._l_hypoPedes.append(hypoPedes)
        #         else:
        #             hypoPedes = Pedestrian(
        #                 idx=99, from_x_m=startPos[0], from_y_m=startPos[1],
        #                 to_x_m=MP[0], to_y_m=MP[1],
        #                 covLong=pCovLon, covLat=pCovLat, vx_ms=pVx,
        #                 startTime=pose.timestamp_s, appearRate=pOtherRate,
        #                 interactRate=interactRate)
        #             self._l_hypoPedes.append(hypoPedes)

        return l_fov_d, hasHypo


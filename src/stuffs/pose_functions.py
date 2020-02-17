from pose import Pose, VehicleDynamic

import numpy as np

import _param as param


def updatePose(lastPose, u_in, dT=param._dT):
    nextTimestamp_s = round(lastPose.timestamp_s + dT, 3)
    vx = lastPose.vdy.vx_ms
    dP = vx * dT + 0.5 * u_in * (dT**2)
    dX = dP * np.cos(lastPose.yaw_rad)
    dY = dP * np.sin(lastPose.yaw_rad)
    nextVDY = VehicleDynamic(vx + u_in * dT, 0)
    next_covLatLong = updateCovLatlong(lastPose.covLatLong, dT, dP, 0)
    nextPose = Pose(
        x_m=lastPose.x_m + dX, y_m=lastPose.y_m + dY,
        yaw_rad=lastPose.yaw_rad, vdy=nextVDY,
        covLatLong=next_covLatLong,
        timestamp_s=nextTimestamp_s
        )
    return nextPose


def updatePoseList(lastPose, u_in, nextTimestamp_s, dT=param._dT, straight=True):
    """
    Update vehicle pose with given longtitude acceleration and timestamp
    Args:
        lastPose: the lastest pose
        u_in: current acceleration as input
        nextTimestamp_s: next timestamp
        straight: bool presents if vehicle go straight
    Return:
        l_pose(timestamp: pose): discreted pose list to given timestamp
        l_u(timestamp: u): input list to given timestamp
    """
    assert nextTimestamp_s > lastPose.timestamp_s

    if not straight:
        return updatePoseTurn(lastPose, u_in, nextTimestamp_s)
    else:
        step = int((nextTimestamp_s - lastPose.timestamp_s)/dT)
        vx = lastPose.vdy.vx_ms
        dyaw = 0
        l_pose = {}
        for k in range(1, step+1, 1):
            dTime = k * dT
            nextT = round(lastPose.timestamp_s + dTime, 3)
            dP = vx * dTime + 0.5 * u_in * (dTime**2)
            dX = dP * np.cos(lastPose.yaw_rad)
            dY = dP * np.sin(lastPose.yaw_rad)
            next_vdy = VehicleDynamic(vx + u_in * dTime, dyaw)
            # update covariance matrix from the nearest pose
            next_covLatLong = 0
            if not l_pose:
                next_covLatLong = updateCovLatlong(lastPose.covLatLong,
                                                   dT, dP, 0)
            else:
                nearestPose = l_pose[max(l_pose)]
                dPNearest = np.linalg.norm([lastPose.x_m+dX-nearestPose.x_m,
                                            lastPose.y_m+dY-nearestPose.y_m])
                next_covLatLong = updateCovLatlong(nearestPose.covLatLong,
                                                   dT, dPNearest, 0)
            nextPose = Pose(x_m=lastPose.x_m + dX, y_m=lastPose.y_m + dY,
                            yaw_rad=lastPose.yaw_rad, vdy=next_vdy,
                            covLatLong=next_covLatLong,
                            timestamp_s=nextT)
            l_pose[nextT] = nextPose
    return l_pose


def updateCovLatlong(lastCovLatLong, dT, dX, dY):
    """
        Update covariance matrix along moving direction
        Args:
            lastCovLatLong: covariance matrix of last pose in latlong
            dT: traveled time
            dX: traveled longtitude
            dY: traveled lattitude
        Return:
            covariance matrix of next pose in latlong
    """
    return np.diag([lastCovLatLong[0, 0] + (param._ALPHA_V_LONG*dX)**2,
                    lastCovLatLong[1, 1] + (param._ALPHA_V_LAT*(dY+0.05))**2])


def updatePoseTurn(lastPose, u_in, nextTimestamp_s):
    """
    Update vehicle pose with given longtitude acceleration and timestamp
    Args:
        lastPose: current pose
        u_in: acceleration as model input
        nextTimestamp_s: next timestamp
    Return:
        l_pose(timestamp, pose): discreted pose list to given timestamp
        l_u(timestamp, u): input list to given timestamp
    """
    assert nextTimestamp_s > lastPose.timestamp_s

    step = int((nextTimestamp_s - lastPose.timestamp_s)/param._dT)
    vx = lastPose.vdy.vx_ms
    dyaw = lastPose.vdy.dyaw_rads
    l_pose = {}
    for i in range(1, step+1, 1):
        dT = i * param._dT
        current_vx = vx + u_in * (dT - param._dT)
        current_yaw_rad = lastPose.yaw_rad + (dT - param._dT) * dyaw
        if abs(dyaw) < 0.0001:
            dX = current_vx * dT * np.cos(current_yaw_rad)
            dY = current_vx * dT * np.sin(current_yaw_rad)
        else:
            dX = (current_vx / dyaw) * np.sin(dyaw*dT + current_yaw_rad) \
                - (current_vx / dyaw) * np.sin(current_yaw_rad)
            dY = (-current_vx / dyaw) * np.cos(dyaw*dT + current_yaw_rad) \
                + (current_vx / dyaw) * np.cos(current_yaw_rad)
        next_vdy = VehicleDynamic(current_vx, dyaw)
        # update covariance matrix from the nearest pose
        next_covLatLong = 0
        if not l_pose:
            next_covLatLong = updateCovLatlong(lastPose.covLatLong,
                                               param._dT, current_vx * dT, 0)
        else:
            nearestPose = l_pose[max(l_pose)]
            next_covLatLong = updateCovLatlong(nearestPose.covLatLong,
                                               param._dT, current_vx * dT, 0)
        nextPose = Pose(x_m=lastPose.x_m + dX, y_m=lastPose.y_m + dY,
                        yaw_rad=lastPose.yaw_rad + dyaw*dT, vdy=next_vdy,
                        covLatLong=next_covLatLong,
                        timestamp_s=lastPose.timestamp_s + dT)
        l_pose[lastPose.timestamp_s + dT] = nextPose
    return l_pose


def interpolatePose(lastPose, nextTimestamp_s):
    """
    Interpolate vehicle pose with given timestamp
    Args:
        lastPose: current pose
        nextTimestamp_s: next timestamp
    Return:
        l_pose(timestamp, pose): discreted pose list to given timestamp
    """
    assert nextTimestamp_s > lastPose.timestamp_s

    step = int((nextTimestamp_s - lastPose.timestamp_s)/param._dT)
    vx, dyaw = lastPose.vdy.vx_ms, lastPose.vdy.dyaw_rads
    l_pose = {}
    for i in range(1, step+1, 1):
        dT = i * param._dT
        if abs(dyaw) < 0.0001:
            dX = vx * dT * np.cos(lastPose.yaw_rad)
            dY = vx * dT * np.sin(lastPose.yaw_rad)
        else:
            dX = (vx / dyaw) * np.sin(dyaw*dT + lastPose.yaw_rad) \
                - (vx / dyaw) * np.sin(lastPose.yaw_rad)
            dY = (-vx / dyaw) * np.cos(dyaw*dT + lastPose.yaw_rad) \
                + (vx / dyaw) * np.cos(lastPose.yaw_rad)
        nextPose = Pose(x_m=lastPose.x_m + dX, y_m=lastPose.y_m + dY,
                        yaw_rad=lastPose.yaw_rad + dyaw*dT,
                        timestamp_s=nextTimestamp_s)
        l_pose[lastPose.timestamp_s + dT] = nextPose
    return l_pose


def rectangle(x_m, y_m, yaw_rad, length, width):
    """
        Return rectangle centered at given position
    """
    rot = np.array([[np.cos(yaw_rad), np.sin(yaw_rad)],
                    [-np.sin(yaw_rad), np.cos(yaw_rad)]])
    trans = np.array([x_m, y_m])
    poly = np.array([[-length/2, -width/2],
                     [length/2, -width/2],
                     [length/2, width/2],
                     [-length/2, width/2]])
    return np.dot(poly, rot) + trans


def minFOVAngle(x_m, y_m, yaw_rad, poly):
    """
    Minimum facing angle of pose(x,y,theta) to a polygon
    """
    vector = np.array([np.cos(yaw_rad), np.sin(yaw_rad)])
    min_angle = 10
    min_vertex = None
    for i, vertex in enumerate(poly):
        p2v = np.array([vertex[0] - x_m, vertex[1] - y_m])
        p2v = p2v / np.linalg.norm(p2v)
        # angle = np.arccos(np.clip(np.dot(vector, p2v), -1.0, 1.0))
        angle = np.arccos(np.dot(vector, p2v))
        if angle < min_angle:
            min_angle = angle
            min_vertex = i
    if min_angle >= np.pi/2:
        min_angle = None
    return poly[min_vertex], min_angle


def distanceToMergePoint(pose, poly, dThres=1):
    """
        EGO ------------------------ MP
             `  alpha/               ^
                `   /                |
                    `                |
         o-------------`o <--dThres->| visibleDistance
         |   obstacle   |  `         |
         |   polygon    |      `     |
         o--------------o          ` V
    """
    randVertex, alpha = minFOVAngle(
        x_m=pose.x_m,
        y_m=pose.y_m,
        yaw_rad=pose.yaw_rad,
        poly=poly
        )
    if alpha is None:
        return None, None, None, None
    else:
        dS = np.sqrt((randVertex[0]-pose.x_m)**2 + (randVertex[1]-pose.y_m)**2)
        heading = np.array([np.cos(pose.yaw_rad), np.sin(pose.yaw_rad)])
        dToMergePoint = dS * np.cos(alpha) + dThres
        MP = heading * dToMergePoint + np.array([pose.x_m, pose.y_m])
        visibleDistance = dToMergePoint * np.tan(alpha)
    return dToMergePoint, MP, visibleDistance, randVertex

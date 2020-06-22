import numpy as np
import math
from shapely.geometry import Polygon, Point
from scipy.spatial import Delaunay

from pose import Pose, VehicleDynamic
import _param as param


def updatePose(lastPose, u_in, dT, updateCov=False):
    vx = lastPose.vdy.vx_ms
    nextVDY = VehicleDynamic(vx + u_in * dT, 0)
    nextTimestamp_s = round(lastPose.timestamp_s + dT, 2)

    dP = vx * dT + 0.5 * u_in * (dT**2)
    dP = max(dP, 0)

    dX = dP * math.cos(lastPose.yaw_rad)
    dY = dP * math.sin(lastPose.yaw_rad)

    if vx == 0:
        next_covLatLong = updateCovLatlong(lastPose.covLatLong, dT, dP, 0)
    elif updateCov:
        next_covLatLong = updateCovLatlong(lastPose.covLatLong, dT, dP, 0)
    else:
        next_covLatLong = lastPose.covLatLong

    nextPose = Pose(x_m=lastPose.x_m + dX, y_m=lastPose.y_m + dY,
                    yaw_rad=lastPose.yaw_rad, vdy=nextVDY,
                    covLatLong=next_covLatLong, timestamp_s=nextTimestamp_s)

    return nextPose


def updatePoseList(lastPose, u_in, nextTimestamp_s, dT):
    """
    Update vehicle pose with given longtitude acceleration and timestamp
    Args:
        lastPose: the lastest pose
        u_in: current acceleration as input
        nextTimestamp_s: next timestamp
    Return:
        l_pose(timestamp: pose): discreted pose list to given timestamp
        l_u(timestamp: u): input list to given timestamp
    """
    # assert nextTimestamp_s > lastPose.timestamp_s
    l_pose = {}
    dyaw = 0
    step = int((nextTimestamp_s - lastPose.timestamp_s)/dT)
    vx = lastPose.vdy.vx_ms

    for k in range(1, step+1, 1):

        dTime = k * dT
        nextT = round(lastPose.timestamp_s + dTime, 2)
        next_vdy = VehicleDynamic(vx + u_in * dTime, dyaw)

        dP = vx * dTime + 0.5 * u_in * (dTime**2)
        dP = max(dP, 0)

        dX = dP * math.cos(lastPose.yaw_rad)
        dY = dP * math.sin(lastPose.yaw_rad)

        # update covariance matrix from the nearest pose
        next_covLatLong = 0
        if not l_pose:
            next_covLatLong = updateCovLatlong(
                lastPose.covLatLong, dT, dP, 0)
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


def updateCovLatlong(lastCovLatLong, dT, dX, dY,
                     alpha_v_long=param._ALPHA_V_LONG,
                     alpha_v_lat=param._ALPHA_V_LAT):
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
    x = (alpha_v_long*dX)**2
    y = (alpha_v_lat*(dY+0.05))**2

    if dX == 0 and dY == 0:
        x = -0.3
        y = -0.3

    covX = max(lastCovLatLong[0, 0] + x, 0)
    covY = max(lastCovLatLong[1, 1] + y, 0)

    return np.diag([covX, covY])


def rectangle(pose, length, width):
    """
    Return rectangle centered at given position
    """
    poly = np.array([[-length/2, -width/2],
                     [length/2, -width/2],
                     [length/2, width/2],
                     [-length/2, width/2]])
    return np.dot(poly, pose.getRotation()) + pose.getTranslation()


def minFOVAngle(pose, poly):
    """
    Minimum facing angle of pose(x,y,theta) to a polygon
    """
    min_angle = 10
    min_vertex = None
    pos = np.array([pose.x_m, pose.y_m])
    pos = pos + pose.heading() * param._CAR_LENGTH * 0.3
    for i, vertex in enumerate(poly):
        p2v = np.array([vertex[0] - pos[0], vertex[1] - pos[1]])
        p2v /= np.linalg.norm(p2v)
        # angle = np.arccos(np.clip(np.dot(vector, p2v), -1.0, 1.0))
        angle = np.arccos(np.dot(pose.heading(), p2v))
        if angle < min_angle:
            min_angle = angle
            min_vertex = i
    if min_angle >= np.pi:
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
        heading = np.array([math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)])
        dToMergePoint = dS * math.cos(alpha) + dThres
        MP = heading * dToMergePoint + np.array([pose.x_m, pose.y_m])
        visibleDistance = dToMergePoint * np.tan(alpha)
    return dToMergePoint, MP, visibleDistance, randVertex


def perp(a):
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b


def onSegment(p, r, q):
    if ((q[0] <= max(p[0], r[0])) and (q[0] >= min(p[0], r[0])) and
       (q[1] <= max(p[1], r[1])) and (q[1] >= min(p[1], r[1]))):
        return True
    return False


def seg_intersect(a1, a2, b1, b2):
    da = a2-a1
    db = b2-b1
    dp = a1-b1
    dap = perp(da)
    denom = np.dot(dap, db)
    num = np.dot(dap, dp)
    cross = (num / denom)*db + b1
    if onSegment(a1, a2, cross) and onSegment(b1, b2, cross):
        return cross
    else:
        return None


def FOV(pose, polys, angle, radius, nrRays=50):
    """
    Field of view (FOV) around ego car
    """
    l_alpha = np.linspace(-angle, angle, nrRays) + pose.yaw_rad
    l_alpha = np.append(l_alpha, np.array([pose.yaw_rad]))
    l_alpha.sort()

    l1_1 = np.array([pose.x_m, pose.y_m])
    l1_1 = l1_1 + pose.heading() * param._CAR_LENGTH * 0.3

    fov_range = 0
    l_fov = np.empty((0, 2))
    l_fov = np.append(l_fov, np.array([l1_1]), axis=0)

    for k, alpha in enumerate(l_alpha):
        direction = np.array([math.cos(alpha), math.sin(alpha)])
        l1_2 = l1_1 + direction * radius
        ip = l1_2

        for poly in polys:
            for i in range(0, poly.shape[0], 1):
                ip_tmp = seg_intersect(l1_1, l1_2, poly[i-1], poly[i])
                if ip_tmp is not None:
                    if np.linalg.norm(ip_tmp-l1_1) < np.linalg.norm(ip-l1_1):
                        ip = ip_tmp + direction * 0.1
        l_fov = np.append(l_fov, np.array([ip]), axis=0)

        if alpha == pose.yaw_rad:
            fov_range = np.linalg.norm(ip - l1_1)

    return l_fov, fov_range


# check if a point lies in a polygon (use Shapely, non-convex)
def inPolyPoint(point, poly):
    point = Point(point)
    return point.within(poly)


# check if any point in list lies in a polygon (use Shapely, non-convex)
def inPolyPointList(pointList, poly):
    for pt in pointList:
        point = Point(pt)
        if point.within(poly):
            return True
    return False


# check if a point lies in a polygon (use scipy, only convex)
def inPolygonPoint(point, poly):
    poly = Delaunay(poly)
    # return np.count_nonzero(poly.find_simplex(point) >= 0) > 0
    return poly.find_simplex(point) >= 0


def orientation(p, q, r):
    # to find the orientation of an ordered triplet (p,q,r)
    # function returns the following values:
    # 0 : Colinear points
    # 1 : Clockwise points
    # 2 : Counterclockwise

    # See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/  
    # for details of below formula.

    val = (float(q[1]-p[1])*(r[0]-q[0]))-(float(q[0]-p[0])*(r[1]-q[1]))
    if (val > 0):
        # Clockwise orientation
        return 1
    elif (val < 0):
        # Counterclockwise orientation
        return 2
    else:
        # Colinear orientation
        return 0


# The main function that returns true if
# the line segment 'p1q1' and 'p2q2' intersect.
def doIntersect(p1, q1, p2, q2):
    # Find the 4 orientations required for
    # the general and special cases
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)
    # General case
    if ((o1 != o2) and (o3 != o4)):
        return True
    # Special Cases
    # p1 , q1 and p2 are colinear and p2 lies on segment p1q1
    if ((o1 == 0) and onSegment(p1, p2, q1)):
        return True
    # p1 , q1 and q2 are colinear and q2 lies on segment p1q1
    if ((o2 == 0) and onSegment(p1, q2, q1)):
        return True
    # p2 , q2 and p1 are colinear and p1 lies on segment p2q2
    if ((o3 == 0) and onSegment(p2, p1, q2)):
        return True
    # p2 , q2 and q1 are colinear and q1 lies on segment p2q2
    if ((o4 == 0) and onSegment(p2, q1, q2)):
        return True
    # If none of the cases
    return False


# ---------------------- BACK UP FUNCTIONS -----------------------------


def computeAccToStop(from_x_m, from_y_m, to_x_m, to_y_m, vx_ms):
    s = np.sqrt((from_x_m-to_x_m)**2 + (from_y_m-to_y_m)**2)
    return - 0.5 * vx_ms**2 / s


def updatePoseTurn(lastPose, u_in, nextTimestamp_s, _dT=param._dT):
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

    step = int((nextTimestamp_s - lastPose.timestamp_s)/_dT)
    vx = lastPose.vdy.vx_ms
    dyaw = lastPose.vdy.dyaw_rads
    l_pose = {}
    for i in range(1, step+1, 1):
        dT = i * _dT
        current_vx = vx + u_in * (dT - _dT)
        current_yaw_rad = lastPose.yaw_rad + (dT - _dT) * dyaw
        if abs(dyaw) < 0.0001:
            dX = current_vx * dT * math.cos(current_yaw_rad)
            dY = current_vx * dT * math.sin(current_yaw_rad)
        else:
            dX = (current_vx / dyaw) * math.sin(dyaw*dT + current_yaw_rad) \
                - (current_vx / dyaw) * math.sin(current_yaw_rad)
            dY = (-current_vx / dyaw) * math.cos(dyaw*dT + current_yaw_rad) \
                + (current_vx / dyaw) * math.cos(current_yaw_rad)
        next_vdy = VehicleDynamic(current_vx, dyaw)
        # update covariance matrix from the nearest pose
        next_covLatLong = 0
        if not l_pose:
            next_covLatLong = updateCovLatlong(lastPose.covLatLong,
                                               _dT, current_vx * dT, 0)
        else:
            nearestPose = l_pose[max(l_pose)]
            next_covLatLong = updateCovLatlong(nearestPose.covLatLong,
                                               _dT, current_vx * dT, 0)
        nextPose = Pose(x_m=lastPose.x_m + dX, y_m=lastPose.y_m + dY,
                        yaw_rad=lastPose.yaw_rad + dyaw*dT, vdy=next_vdy,
                        covLatLong=next_covLatLong,
                        timestamp_s=lastPose.timestamp_s + dT)
        l_pose[lastPose.timestamp_s + dT] = nextPose
    return l_pose

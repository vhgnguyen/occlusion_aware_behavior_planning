from scipy.stats import mvn

import numpy as np
import math

import _param as param
import gaussian as gaussian


def collisionEventSeverity(ego_vx, obj_vx, method="sigmoid"):
    """
    Collision event severity of ego vehicle with another object
    Args:
        ego_vx: longtitude velocity vector of vehicle in UTM
        obj_vx: longtitude velocity vector of object in UTM
        method: one of these ["constant", "linear",
                                "quadratic", "sigmoid",
                                "mass"]
    Return:
        severity: collision event severity
    """
    assert ego_vx.shape == (2,) and obj_vx.shape == (2,)

    relativeVelocity = ego_vx - obj_vx
    severity = 0.0
    if method == "constant":
        severity = param._SEVERITY_MIN_WEIGHT_CONST
    elif method == "linear":
        severity = np.linalg.norm(relativeVelocity)
    elif method == "quadratic":
        severity = np.linalg.norm(relativeVelocity)**2
        severity *= param._SEVERITY_QUAD_WEIGHT
        severity += param._SEVERITY_MIN_WEIGHT_CONST
    elif method == "sigmoid":
        severity = param._SEVERITY_SIG_MAX
        severity /= (1.0 + np.exp(- param._SEVERITY_SIG_AVG_VX
                     * np.linalg.norm(relativeVelocity)))
    else:
        severity = param._SEVERITY_MIN_WEIGHT_CONST

    return severity


def collisionIndicatorComputeSimple(bound, dMean, dCov):
    """
    Risk indicator function for collision
        by CDF of distance's PDF between two objects over minkowski polygon
        in orthogonal or parallel case
    Param:
        bound: min/max distance from (0, 0) to polygon's vertices
        dMean: (2,) vector mean distance between two objects
        dCov: (2,2) array covariance matrix of distance
    Return:
        I: (float) value of collision indicator
    """
    assert bound['max'].shape == (2,) and bound['min'].shape == (2,)
    assert dMean.shape == (2,)
    assert dCov.shape == (2, 2)

    # quick check for long distance
    if (abs(dMean) > (abs(bound['max']) + np.diag(dCov))).all():
        return 0
    else:
        prob, _ = mvn.mvnun(bound['min'], bound['max'], dMean, dCov)
        return prob


def collisionIndicatorCompute(poly, bound, dMean, dCov):
    """
    Risk indicator function for collision
        by CDF of distance's PDF between two objects over minkowski polygon
    Param:
        poly: (n,2) array of minkowski polygon vertices
        bound: min/max distance from (0, 0) to polygon's vertices
        dMean: (2,) vector mean distance between two objects
        dCov: (2,2) array covariance matrix of distance
    Return:
        I: (float) value of collision indicator
    """
    assert poly.ndim == 2 and poly.shape[1] == 2
    assert dMean.shape == (2,)
    assert dCov.shape == (2, 2)

    # quick check for long distance
    if (abs(dMean) > (abs(bound['max']) + np.diag(dCov))).all():
        return 0
    else:
        return gaussian.polyIntegratePdf(poly, dMean, dCov)


def collisionEventRate(collisionIndicator,
                       eventRate_max=param._COLLISION_RATE_MAX,
                       eventRate_beta=param._COLLISION_RATE_BETA):
    """
    Function to calculate event rate
    Args:
        eventRate_max: maximal event rate
        eventRate_beta: slope weight
        collisionIndicator: indicator factor between [0,1]
    Return: (float) collision event rate
    """
    assert np.isscalar(eventRate_max) and eventRate_max >= 1.0
    assert np.isscalar(eventRate_beta) and eventRate_beta > 0.0
    assert np.isscalar(collisionIndicator) and 0.0 <= collisionIndicator <= 1.0
    return eventRate_max \
        * (1.0 - np.exp(-eventRate_beta*collisionIndicator)) \
        / (1.0 - np.exp(-eventRate_beta))


def collisionRisk(egoPose, egoPoly, objPose, objPoly):
    """
    Risk function for collision between ego vehicle and moving object
    Param:
        egoPose: ego vehicle
        objPose: pose of object
    Return:
        col_risk: (float) collision risk cost in a time point
        col_rate: (float) collision rate of the event
        col_indicator: (float) collision indicator between two object
    """
    poly, bound = gaussian.minkowskiSum(egoPoly, objPoly)
    dMean = np.array([egoPose.x_m-objPose.x_m,
                      egoPose.y_m-objPose.y_m])
    dCov = egoPose.covUtm + objPose.covUtm
    diff_yaw = abs(egoPose.yaw_rad-objPose.yaw_rad)
    col_indicator = 0

    # handle parallel and orthogonal case
    if abs(math.remainder(diff_yaw, np.pi/2)) < param._COLLISION_ORTHO_THRES:
        col_indicator = collisionIndicatorComputeSimple(bound, dMean, dCov)
    # handle general case
    else:
        col_indicator = collisionIndicatorCompute(
            poly=poly,
            bound=bound,
            dMean=dMean,
            dCov=dCov)

    col_severity = collisionEventSeverity(
        ego_vx=egoPose.vxUtm,
        obj_vx=objPose.vxUtm,
        method="quadratic")
    col_rate = collisionEventRate(
        collisionIndicator=col_indicator)
    col_risk = col_rate*col_severity

    return col_risk, col_rate, col_indicator


def unseenEventRate(unseenEventIndicator,
                    eventRate_max=param._UNSEENEVENT_RATE_MAX,
                    eventRate_beta=param._UNSEENEVENT_RATE_BETA):
    assert np.isscalar(eventRate_max) and eventRate_max >= 1.0
    assert np.isscalar(eventRate_beta) and eventRate_beta > 0.0
    assert 0.0 <= unseenEventIndicator <= 1.0
    return eventRate_max \
        * (1.0 - np.exp(-eventRate_beta*unseenEventIndicator)) \
        / (1.0 - np.exp(-eventRate_beta))


def unseenEventRisk(d2MP, ego_vx, ego_acc, dVis,
                    obj_vx=param._V_MAX_OBJECT, dBrakeThresh=0.3,
                    ref_deAcc=-0.5):
    # ego vehicle brake distance
    sBrake_max = abs(0.5 * ego_vx**2 / param._A_MAX_BRAKE)

    col_indicator = 0
    if d2MP <= sBrake_max + dBrakeThresh:
        t_obj2MP = dVis / obj_vx
        t_ego2Brake = - ego_vx / param._A_MAX_BRAKE
        v_egoAtMP = max(np.sqrt(ego_vx**2 + 2*param._A_MAX_BRAKE*d2MP), 0)

        col_indicator = 1 - np.exp(-5*min(t_ego2Brake / t_obj2MP, 1))
        col_indicator = min(t_ego2Brake / t_obj2MP, 1)
        col_event = unseenEventRate(col_indicator)

        col_severity = 0.05 * abs(v_egoAtMP)**2
        col_risk = col_event * col_severity

        return col_indicator, col_event, col_risk
    else:
        # ideal velocity for comfort stopping at merge point
        v_ref = min(np.sqrt(2*(d2MP - dBrakeThresh)*abs(ref_deAcc)), param._C_V_CRUISE)
        col_risk = 0.05 * abs(ego_vx - v_ref)**2 * sBrake_max / d2MP

        return col_indicator, 0, col_risk


def unseenObjectEventRate(d2MP, ego_vx, ego_acc, dVis, brakeD=0.3):

    # object 
    t_obj2MP = dVis / param._V_MAX_OBJECT
    sBrake_obj_max = abs(0.5 * param._V_MAX_OBJECT / param._A_MAX_BRAKE) - 1 # offset

    # ego vehicle
    t_ego2MP = min(abs(np.roots([0.5*ego_acc, ego_vx, -d2MP])))
    s_obj2MP = t_ego2MP * param._V_MAX_OBJECT #  assume object travel
    sBrake_max = abs(0.5 * ego_vx**2 / param._A_MAX_BRAKE) + brakeD # safe distance for brake

    v_ego_max = np.sqrt(max(d2MP - sBrake_max, 0) * abs(param._A_MAX_BRAKE))
    v_ego_max = min(v_ego_max, 6)
    # print("T_MP:{:.2f}, Vmax:{:.2f}".format(t_ego2MP, v_ego_max))
    # print("T_OBJ_MP", t_obj2MP)
    # probability of unseen object move close to vehicle at MP
    _UNSEEN_WEIGHT = 0.2
    if dVis > t_ego2MP*param._V_MAX_OBJECT + sBrake_obj_max:
        probUnseen = 0
    else:
        probUnseen = np.exp(-_UNSEEN_WEIGHT * abs(dVis - d2MP))
        # TODO: for this case acc should be non increased

    # probability of collision if object appear

    # severity of collision

    unseenSeverity = 0.01 * abs(2 - ego_vx)

    unseenRisk = probUnseen * unseenSeverity

    return probUnseen, unseenRisk


def testUnseen(d2MP, ego_vx, ego_acc, dVis, brakeD=2):

    _MAX_UNSEEN_OBJECT = 2
    _OBJECT_APPEAR_WEIGHT = 5

    time2MergePoint = min(abs(np.roots([0.5*ego_acc, ego_vx, -d2MP])))
    probUnseen = 1 - np.exp(-_MAX_UNSEEN_OBJECT * time2MergePoint)

    sBrake_max = abs(0.5 * ego_vx**2 / param._A_MAX_BRAKE) + brakeD

    tObjectMP = dVis / param._V_MAX_OBJECT

    unseenCost = np.exp(-0.5 * time2MergePoint/tObjectMP) * 0.05
    return probUnseen, unseenCost

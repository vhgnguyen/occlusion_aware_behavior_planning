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
        # return gaussian.polyIntegratePdf(poly, dMean, dCov)
        return gaussian.polyIntegratePdf(poly, dMean, dCov, method='simulation')


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


def collisionEventRate(collisionIndicator,
                       tau_inv_max=param._COLLISION_RATE_MAX,
                       eventRate_beta=param._COLLISION_RATE_BETA):
    """
    Function to calculate event rate
    Args:
        tau_inv_max: maximal event rate
        eventRate_beta: slope weight
        collisionIndicator: indicator factor between [0,1]
    Return: (float) collision event rate
    """
    assert np.isscalar(tau_inv_max) and tau_inv_max >= 1.0
    assert np.isscalar(eventRate_beta) and eventRate_beta > 0.0
    assert np.isscalar(collisionIndicator) and 0.0 <= collisionIndicator <= 1.0
    return tau_inv_max \
        * (1.0 - np.exp(-eventRate_beta*collisionIndicator)) \
        / (1.0 - np.exp(-eventRate_beta))


def unseenObjectEventRate(dToMergePoint, ego_vx, ego_acc, visibleD, brakeD=2):
    """
    Event rate of unseen obstacles appears from limited view
    Args:
    Return:
    """
    # some fix constants:
    _MAX_UNSEEN_OBJECT = 2
    _MAX_UNSEEN_EVENT_RATE = 3
    _OBJECT_APPEAR_WEIGHT = 0.01
    _UNSEEN_SEVERITY_MAX = 0.1

    # probability of object appear until reaching the merge point
    time2MergePoint = min(abs(np.roots([0.5*ego_acc, ego_vx, -dToMergePoint])))
    probUnseen = 1 - np.exp(-_MAX_UNSEEN_OBJECT * time2MergePoint)
    safeD = time2MergePoint * param._V_MAX_OBJECT
    probObj = np.exp(-_OBJECT_APPEAR_WEIGHT*min(visibleD/safeD, 1))
    probUnseen *= probObj

    # event rate
    tau_unseen = _MAX_UNSEEN_EVENT_RATE * probUnseen

    # severity model for collision with an unseen object
    sBrake_max = 0.5 * ego_vx**2 / param._A_MAX_BRAKE + brakeD
    unseenSeverity = (1 - min(dToMergePoint / sBrake_max, 1)) * _UNSEEN_SEVERITY_MAX

    # risk
    unseenRisk = tau_unseen * unseenSeverity

    return tau_unseen, unseenSeverity, unseenRisk

def testUnseen(dToMergePoint, ego_vx, ego_acc, visibleD, brakeD=2):

    _MAX_UNSEEN_OBJECT = 2
    _OBJECT_APPEAR_WEIGHT = 5

    time2MergePoint = min(abs(np.roots([0.5*ego_acc, ego_vx, -dToMergePoint])))
    probUnseen = 1 - np.exp(-_MAX_UNSEEN_OBJECT * time2MergePoint)

    sBrake_max = abs(0.5 * ego_vx**2 / param._A_MAX_BRAKE) + brakeD

    tObjectMP = visibleD / param._V_MAX_OBJECT

    unseenCost = np.exp(-0.5 * time2MergePoint/tObjectMP) * 0.1
    return probUnseen, unseenCost


def testUnseen1(dToMergePoint, ego_vx, ego_acc, visibleD, brakeD=1):

    _MAX_UNSEEN_OBJECT = 2
    _SEVERITY_CONST = 10
    t_ego2MP = min(abs(np.roots([0.5*ego_acc, ego_vx, -dToMergePoint])))
    print(t_ego2MP)
    t_obj2MP = visibleD / param._V_MAX_OBJECT
    s_obj2MP_assume = t_ego2MP * param._V_MAX_OBJECT

    sBrake_max = abs(0.5 * ego_vx**2 / param._A_MAX_BRAKE) + brakeD
    d_ego_objMP = 0.5*t_obj2MP**2*ego_acc + ego_vx*t_obj2MP

    if s_obj2MP_assume < visibleD:
        unseenSeverity = 0
    else:
        unseenSeverity = 0.1 * (sBrake_max + d_ego_objMP) / dToMergePoint

    _UNSEEN_WEIGHT = 5
    probUnseen = np.exp(-_UNSEEN_WEIGHT * visibleD / s_obj2MP_assume)
    unseenRisk = probUnseen * unseenSeverity

    return probUnseen, unseenRisk


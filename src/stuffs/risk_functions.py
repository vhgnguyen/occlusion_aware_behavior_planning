from scipy.stats import mvn

import numpy as np
import math

import _param as param
import gaussian as gaussian


def collisionEventSeverity(ego_vx, obj_vx, method="quadratic", gom_rate=0.1, gom_vx=5):
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
    # assert ego_vx.shape == (2,) and obj_vx.shape == (2,)

    dv = ego_vx - obj_vx
    severity = 0.0
    if method == "constant":
        severity = param._SEVERITY_MIN_WEIGHT_CONST
    elif method == "linear":
        severity = np.linalg.norm(dv)
    elif method == "quadratic":
        severity = np.linalg.norm(dv)**2
        severity *= param._SEVERITY_QUAD_WEIGHT
        severity += param._SEVERITY_MIN_WEIGHT_CONST
    elif method == "sigmoid":
        severity = param._SEVERITY_SIG_MAX
        sig_dv = np.linalg.norm(dv) - param._SEVERITY_SIG_AVG_VX
        severity /= (1.0 + np.exp(-param._SEVERITY_SIG_B * sig_dv))
        severity += param._SEVERITY_MIN_WEIGHT_CONST
    elif method == "gompertz":
        # gom_dv = np.linalg.norm(dv) - gom_vx
        gom_dv = np.linalg.norm(dv)
        severity = param._SEVERITY_GOM_MAX*np.exp(-5*np.exp(-gom_rate*(gom_dv)))
        severity += param._SEVERITY_MIN_WEIGHT_CONST
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
    # assert bound['max'].shape == (2,) and bound['min'].shape == (2,)
    # assert dMean.shape == (2,)
    # assert dCov.shape == (2, 2)

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
    # assert poly.ndim == 2 and poly.shape[1] == 2
    # assert dMean.shape == (2,)
    # assert dCov.shape == (2, 2)

    # quick check for long distance
    if (abs(dMean) > (abs(bound['max']) + np.diag(dCov))).all():
        return 0
    else:
        return gaussian.polyIntegratePdf(poly, dMean, dCov)


def collisionIndicator(egoPose, egoPoly, objPose, objPoly):
    """
    Indicator function for collision between ego vehicle and moving object
    Param:
        egoPose: ego vehicle
        objPose: pose of object
    Return:
        col_indicator: (float) collision indicator between two object
    """
    dMean = np.array([egoPose.x_m-objPose.x_m,
                      egoPose.y_m-objPose.y_m])
    dCov = egoPose.covUtm + objPose.covUtm
    diff_yaw = abs(egoPose.yaw_rad-objPose.yaw_rad)
    col_indicator = 0

    # handle parallel and orthogonal case
    if abs(math.remainder(diff_yaw, np.pi/2)) < param._COLLISION_ORTHO_THRES:
        poly, bound = gaussian.minkowskiSumOrthogonal(egoPoly, objPoly)
        col_indicator = collisionIndicatorComputeSimple(bound, dMean, dCov)
    # handle general case
    else:
        poly, bound = gaussian.minkowskiSum(egoPoly, objPoly)
        col_indicator = collisionIndicatorCompute(
            poly=poly,
            bound=bound,
            dMean=dMean,
            dCov=dCov)

    return col_indicator


def collisionEventRate(collisionIndicator, method='exponential',
                       eventRate_max=param._COLLISION_RATE_MAX,
                       exp_beta=param._COLLISION_RATE_EXP_BETA,
                       sig_beta=param._COLLISION_RATE_SIG_BETA):
    """
    Function to calculate event rate
    Args:
        eventRate_max: maximal event rate
        eventRate_beta: slope weight
        collisionIndicator: indicator factor between [0,1]
    Return: (float) collision event rate
    """
    # assert np.isscalar(eventRate_max) and eventRate_max >= 1.0
    # assert np.isscalar(eventRate_beta) and eventRate_beta > 0.0
    # assert np.isscalar(collisionIndicator) and 0.0 <= collisionIndicator <= 1.0
    if method == 'exponential':
        return eventRate_max \
            * (1.0 - np.exp(-exp_beta*collisionIndicator)) \
            / (1.0 - np.exp(-exp_beta))
    if method == 'sigmoid':
        return eventRate_max / (1.0 + np.exp(- sig_beta * (collisionIndicator - 0.5)))


def collisionRisk(col_severity, col_rate):
    """
    Risk function for collision between ego vehicle and moving object
    Param:
        col_rate: (float) collision rate of the event
        col_indicator: (float) collision indicator between two object
    Return:
        col_risk: (float) collision risk cost in a time point
    """

    return col_rate * col_severity


def appearProbability(ego_vx, ego_acc, dS, rate):
    t = min(abs(np.roots([0.5*ego_acc, ego_vx, -dS])))
    return 1 - np.exp(- rate * t)


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
                    ref_deAcc=-0.5,
                    objectAppearRate=0.1):
    # ego vehicle brake distance
    sBrake_max = abs(0.5 * ego_vx**2 / param._A_MAX_BRAKE)
    t_ego2MP = min(abs(np.roots([0.5*ego_acc, ego_vx, -d2MP])))
    pro_unseen = 1 - np.exp(- objectAppearRate * t_ego2MP)

    col_indicator = 0
    if d2MP <= sBrake_max + dBrakeThresh:
        t_obj2MP = dVis / obj_vx
        t_ego2Brake = - ego_vx / param._A_MAX_BRAKE
        v_max2 = max(ego_vx**2 + 2*param._A_MAX_BRAKE*d2MP, 0)
        v_egoAtMP = max(np.sqrt(v_max2), 0)

        col_indicator = min(t_ego2Brake / t_obj2MP, 1) * pro_unseen
        col_event = unseenEventRate(col_indicator)

        col_severity = 0.05 * abs(v_egoAtMP)**2
        col_risk = col_event * col_severity

        return col_indicator, col_event, col_risk
    else:
        # ideal velocity for comfort stopping at merge point
        v_ref = min(np.sqrt(2*(d2MP - dBrakeThresh)*abs(ref_deAcc)), param._C_V_CRUISE)
        col_risk = 0.01 * abs(ego_vx - v_ref)**2 * pro_unseen

        return col_indicator, 0, col_risk

from scipy.stats import mvn

import numpy as np
import math

import _param as param
import gaussian as gaussian


def collisionEventSeverity(ego_vx, obj_vx, method='sigmoid', gom_rate=1,
                           min_weight=param._SEVERITY_MIN_WEIGHT_CONST,
                           quad_weight=param._SEVERITY_QUAD_WEIGHT,
                           sig_vx=param._SEVERITY_SIG_AVG_VX,
                           sig_max=param._SEVERITY_SIG_MAX,
                           sig_beta=param._SEVERITY_SIG_B,
                           gom_vx=param._SEVERITY_GOM_AVG_VX,
                           gom_max=param._SEVERITY_GOM_MAX,
                           gom_beta=param._SEVERITY_GOM_BETA):
    """
    Collision event severity of ego vehicle with another object
    Args:
        ego_vx: longtitude velocity vector of vehicle in UTM
        obj_vx: longtitude velocity vector of object in UTM
        method: one of these ['constant', 'linear',
                                'quadratic', 'sigmoid', 'gompertz']
    Return:
        severity: collision event severity
    """
    # assert ego_vx.shape == (2,) and obj_vx.shape == (2,)

    dv = ego_vx - obj_vx
    severity = 0.0
    if method == 'constant':
        severity = min_weight
    elif method == 'linear':
        severity = np.linalg.norm(dv)
    elif method == 'quadratic':
        severity = np.linalg.norm(dv)**2
        severity *= quad_weight
        severity += min_weight
    elif method == 'sigmoid':
        sig_dv = np.linalg.norm(dv) - sig_vx
        severity = sig_max
        severity /= (1.0 + np.exp(-sig_beta * sig_dv))
        severity += min_weight
    elif method == 'gompertz':
        gom_dv = np.linalg.norm(dv) - gom_vx
        severity = gom_max
        severity *= np.exp(-gom_beta*np.exp(-gom_rate*gom_dv))
        severity += min_weight
    else:
        severity = min_weight

    return severity


def collisionSeverityHypoVeh(ego_vx, obj_vx, method='sigmoid',
                             quad_weight=param._SEVERITY_QUAD_WEIGHT,
                             min_weight=param._SEVERITY_HYPOVEH_MIN_WEIGHT,
                             sig_max=param._SEVERITY_HYPOVEH_SIG_MAX,
                             sig_avg_vx=param._SEVERITY_HYPOVEH_AVG_VX,
                             sig_beta=param._SEVERITY_HYPOVEH_SIG_B):
    """
    Collision event severity of ego vehicle with hypthetical vehicle
    Args:
        ego_vx: longtitude velocity vector of vehicle in UTM
        obj_vx: longtitude velocity vector of object in UTM
        method: one of these ['constant', 'quadratic', 'sigmoid']
    Return:
        severity: collision event severity
    """
    dv = ego_vx - obj_vx
    severity = 0.0
    if method == 'quadratic':
        severity = np.linalg.norm(dv)**2
        severity *= quad_weight
        severity += min_weight
    elif method == 'sigmoid':
        severity = sig_max
        sig_dv = np.linalg.norm(dv) - sig_avg_vx
        severity /= (1.0 + np.exp(-sig_beta * sig_dv))
        severity += min_weight
    else:
        severity = min_weight

    return severity


def collisionSeverityHypoPedes(ego_vx, obj_vx, method='gompertz',
                               min_weight=param._SEVERITY_HYPOPEDES_MIN_WEIGHT,
                               avg_vx=param._SEVERITY_HYPOPEDES_AVG_VX,
                               sig_max=param._SEVERITY_HYPOPEDES_SIG_MAX,
                               sig_beta=param._SEVERITY_HYPOPEDES_SIG_BETA,
                               gom_max=param._SEVERITY_HYPOPEDES_GOM_MAX,
                               gom_beta=param._SEVERITY_HYPOPEDES_GOM_BETA):
    """
    Collision event severity of ego vehicle with hypthetical vehicle
    Args:
        ego_vx: longtitude velocity vector of vehicle in UTM
        obj_vx: longtitude velocity vector of object in UTM
        method: one of these ['constant', 'gompertz', 'sigmoid']
    Return:
        severity: collision event severity
    """
    dv = ego_vx - obj_vx
    severity = 0.0
    if method == 'sigmoid':
        sig_dv = np.linalg.norm(dv) - avg_vx
        severity = sig_max
        severity /= (1.0 + np.exp(-sig_beta*sig_dv))
        severity += min_weight
    elif method == 'gompertz':
        gom_dv = np.linalg.norm(dv) - avg_vx
        severity = gom_max
        severity *= np.exp(-gom_beta*np.exp(-gom_dv))
        severity += min_weight
    else:
        severity = min_weight

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
    if (dMean > 10).all():
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
    if (dMean > 10).all():
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


def collisionEventRate(collisionIndicator,
                       eventRate_max, exp_beta=param._COLLISION_RATE_EXP_BETA,
                       sig_beta=param._COLLISION_RATE_SIG_BETA,
                       method='exponential'):
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
        t = max(collisionIndicator, 0.005)
        return eventRate_max / (1 + (t / (1-t))**(-sig_beta))


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


def interactRate(a, b=param._AWARENESS_DISTANCE, k=1):
    return 1 - 1 / (1 + np.exp(k * (a-b)))


def limitViewRisk(fov_range, ego_vx, aBrake, dBrake, stdLon, tReact,
                  rateMax=param._FOV_EVENTRATE_MAX,
                  rateBeta=param._FOV_EVENTRATE_BETA,
                  severity_min_weight=param._FOV_SEVERITY_MIN,
                  severity_weight=param._FOV_SEVERITY_WEIGHT):
    ego_vx2 = ego_vx**2
    dReact = ego_vx*tReact
    dSafe = dBrake + stdLon + dReact
    sBrake_max = abs(0.5 * ego_vx2 / aBrake) + dSafe

    # event rate
    eventRate = rateMax
    eventRate *= 1 - 1 / (1 + rateBeta * np.exp(-(fov_range - sBrake_max)))
    # severity
    v_max2 = ego_vx2 + 2*aBrake*(fov_range - dReact)
    severity = severity_weight * max(v_max2, 0)
    severity += severity_min_weight
    # risk
    risk = eventRate * severity

    return eventRate, risk

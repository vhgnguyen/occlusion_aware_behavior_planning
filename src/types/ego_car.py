from matplotlib.patches import Ellipse, Polygon
from scipy import optimize
from shapely.geometry import Polygon
import matplotlib.pyplot as plt
import numpy as np

import pose_functions as pfnc
import risk_functions as rfnc
import _param as param


class EgoVehicle:
    """
    Class define vehicle properties
    Params:
        _idx: vehicle index
        _u: current acceleration [m/s2] as input
        _p_pose(timestamp: pose): future path along the prediction horizon
        _p_u: predicted acceleration input along the prediction horizon
        _p_col_cost(timestamp, col_rate, col_risk): along prediction
        _l_pose(timestamp: pose): traveled poses list
        _l_u(timestamp: u): recorded input during travel
        _is_moving: if vehicle is started or not
    """

    def __init__(self, length, width, env, startPose, u_in):
        self.length = length
        self.width = width

        # prediction params
        self._p_u = None
        self._p_pose = {}
        self._p_eventRate = {}

        # current state
        self._currentPose = startPose
        self._u = u_in
        self._minColValue = 0
        self._minRiskValue = 0

        # optimization state machine
        self._stopState = False
        self._defaultState = True
        self._emergencyState = False
        self._TTB = 0
        self._brake = False

        # environment information
        self._env = env
        self._fov = None
        self._fovRange = None
        self._l_currentObject = {}

        # moved poses
        self._l_u = {startPose.timestamp_s: u_in}
        self._l_pose = {startPose.timestamp_s: startPose}

    def getCurrentPose(self):
        return self._currentPose

    def getCurrentLongtitudeVelocity(self):
        return self._currentPose.vdy.vx_ms

    def getCurrentLongtitudeAcceleration(self):
        return self._u

    def getCurrentTimestamp(self):
        return self._currentPose.timestamp_s

    def getPoly(self, timestamp_s: float):
        """
        Return the bounding polygon of vehicle
        """
        if timestamp_s in self._l_pose:
            pose = self._l_pose[timestamp_s]
            return pfnc.rectangle(pose, self.length, self.width)
        else:
            print("Ego: No state at: ", timestamp_s)
            return None

    def getCurrentPoly(self):
        return pfnc.rectangle(self._currentPose, self.length, self.width)

    def getPredictPoly(self, timestamp_s: float):
        if timestamp_s in self._p_pose:
            pose = self._p_pose[timestamp_s]
            return pfnc.rectangle(pose, self.length, self.width)
        else:
            print("Ego: No predict state at: ", timestamp_s)
            return None

    def getPoseAt(self, timestamp_s: float):
        if timestamp_s in self._l_pose:
            return self._l_pose[timestamp_s]
        else:
            print("Ego: No pose at: ", timestamp_s)
            return None

    # ------------------- Opt function ---------------------

    def _searchEnvironment(self):
        """
        Scan environment at current state
        """
        currentPose = self.getCurrentPose()
        self._l_currentObject, self._fov, self._fovRange = self._env.update(
            pose=currentPose,
            from_timestamp=currentPose.timestamp_s,
            u_in=self._u, radius=param._SCAN_RADIUS,
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

    def _predict(self, u_in: float, dT=param._PREDICT_STEP,
                 predictTime=param._PREDICT_TIME):
        """
        Predict the vehicle motion
        Args:
            u_in: input acceleration
            predictTime: prediction duration
        Return:
            Add the prediction acceleration and pose to prediction list
        """
        self._p_u = None
        self._p_pose = {}
        lastPose = self.getCurrentPose()
        self._p_pose = pfnc.updatePoseList(
            lastPose=lastPose,
            u_in=u_in,
            nextTimestamp_s=lastPose.timestamp_s + predictTime,
            dT=dT
            )
        self._p_u = u_in

    def _escapeRate(self, r=param._ESCAPE_RATE):
        """
        Return escape rate
        """
        return r

    def _riskCost(self, timestamp_s: float, u_in: float,
                  useAwarenessRate, useFOV,
                  minCollisionBrakeVehicle=param._MIN_COL_BRAKE_VEHICLE,
                  minCollisionBrakePedes=param._MIN_COL_BRAKE_PEDESTRIAN):
        """
        Compute collision risk & event rate at given predict timestamp
        """
        currentTime = self.getCurrentTimestamp()
        timestamp_s = round(timestamp_s, 2)
        egoPose = self._p_pose[timestamp_s]
        egoPoly = self.getPredictPoly(timestamp_s)
        l_obj = self._l_currentObject

        total_risk = 0
        total_eventRate = 0

        if useFOV:
            limitViewEvent, limitViewRisk = rfnc.limitViewRisk(
                fov_range=self._fovRange, ego_vx=egoPose.vdy.vx_ms,
                aBrake=param._A_MIN, dBrake=param._D_BRAKE_MIN,
                stdLon=np.sqrt(egoPose.covLatLong[0, 0]),
                tReact=param._T_BRAKE,
                rateMax=param._FOV_EVENTRATE_MAX,
                rateBeta=param._FOV_EVENTRATE_BETA,
                severity_min_weight=param._FOV_SEVERITY_MIN,
                severity_weight=param._FOV_SEVERITY_WEIGHT)

            total_risk += limitViewRisk
            total_eventRate += limitViewEvent

        egoPolygon = Polygon(egoPoly + egoPose.heading())

        for sobj in l_obj['staticObject']:
            sPolygon = Polygon(sobj._poly)
            if egoPolygon.intersects(sPolygon):
                sO_eventRate = param._COLLISION_RATE_MAX
                sO_severity = rfnc.collisionEventSeverity(
                    ego_vx=egoPose.vdy.vx_ms, obj_vx=0,
                    method=param._COLLISION_SEVERITY_MODEL)

                total_eventRate += sO_eventRate
                total_risk += sO_eventRate*sO_severity

                self._minColValue = 1
                if timestamp_s <= currentTime + self._TTB:
                    self._brake = True

        for staticVeh in l_obj['staticVehicle']:
            _, vehPoly = staticVeh.getPredictAt(timestamp_s)
            vehPolygon = Polygon(vehPoly)
            if egoPolygon.intersects(vehPolygon):
                sV_eventRate = param._COLLISION_RATE_MAX
                sV_severity = rfnc.collisionEventSeverity(
                    ego_vx=egoPose.vdy.vx_ms, obj_vx=0,
                    method=param._COLLISION_SEVERITY_MODEL)
                total_eventRate += sV_eventRate
                total_risk += sV_eventRate*sV_severity

                self._minColValue = 1
                if timestamp_s <= currentTime + self._TTB:
                    self._brake = True

        for veh in l_obj['vehicle']:
            vehPose, vehPoly = veh.getPredictAt(timestamp_s)

            vcol_indicator = rfnc.collisionIndicator(
                egoPose=egoPose, egoPoly=egoPoly,
                objPose=vehPose, objPoly=vehPoly)

            vcol_rate = rfnc.collisionEventRate(
                collisionIndicator=vcol_indicator,
                eventRate_max=param._COLLISION_RATE_MAX,
                method=param._COLLISION_EVENT_RATE_MODEL,
                exp_beta=param._COLLISION_RATE_EXP_BETA)

            vcol_severity = rfnc.collisionEventSeverity(
                ego_vx=egoPose.vdy.vx_ms, obj_vx=vehPose.vdy.vx_ms,
                method=param._COLLISION_SEVERITY_MODEL)

            vcol_risk = rfnc.collisionRisk(
                col_severity=vcol_severity,
                col_rate=vcol_rate)

            self._minColValue = max(self._minColValue, vcol_indicator)
            if timestamp_s <= currentTime + self._TTB:
                if vcol_indicator > minCollisionBrakeVehicle:
                    self._brake = True

            total_risk += vcol_risk
            total_eventRate += vcol_rate
            veh.setCollisionProb(vcol_indicator)

        for pedes in l_obj['pedestrian']:
            pPose, pPoly = pedes.getPredictAt(timestamp_s)

            pcol_indicator = rfnc.collisionIndicator(
                egoPose=egoPose, egoPoly=egoPoly,
                objPose=pPose, objPoly=pPoly)

            pcol_rate = rfnc.collisionEventRate(
                collisionIndicator=pcol_indicator,
                eventRate_max=param._COLLISION_RATE_MAX,
                method=param._COLLISION_EVENT_RATE_MODEL,
                exp_beta=param._COLLISION_RATE_EXP_BETA_PEDES)

            pcol_severity = rfnc.collisionEventSeverity(
                ego_vx=egoPose.vdy.vx_ms, obj_vx=pPose.vdy.vx_ms,
                method=param._COLLISION_SEVERITY_MODEL)

            pcol_risk = rfnc.collisionRisk(
                col_severity=pcol_severity,
                col_rate=pcol_rate)

            self._minColValue = max(self._minColValue, pcol_indicator)
            if timestamp_s <= currentTime + self._TTB:
                if pcol_indicator > minCollisionBrakePedes:
                    self._brake = True

            total_risk += pcol_risk
            total_eventRate += pcol_rate
            pedes.setCollisionProb(pcol_indicator)

        for hypoPedes in l_obj['hypoPedestrian']:
            hPose, hPoly = hypoPedes.getPredictAt(timestamp_s)

            hpcol_indicator = rfnc.collisionIndicator(
                egoPose=egoPose, egoPoly=egoPoly,
                objPose=hPose, objPoly=hPoly)

            hpcol_rate = rfnc.collisionEventRate(
                collisionIndicator=hpcol_indicator*hypoPedes._appearRate,
                eventRate_max=param._COLLISION_HYPOPEDES_RATE_MAX,
                method=param._EVENT_RATE_HYPOPEDES_MODEL,
                exp_beta=param._EVENT_RATE_HYPOPEDES_EXP_BETA,
                sig_beta=param._EVENT_RATE_HYPOPEDES_SIG_BETA)
            if useAwarenessRate:
                hpcol_rate *= hypoPedes._interactRate

            hpcol_severity = rfnc.collisionSeverityHypoPedes(
                ego_vx=egoPose.vdy.vx_ms, obj_vx=hPose.vdy.vx_ms,
                method=param._SEVERITY_HYPOPEDES_MODEL,
                min_weight=param._SEVERITY_HYPOPEDES_MIN_WEIGHT,
                avg_vx=param._SEVERITY_HYPOPEDES_AVG_VX,
                sig_max=param._SEVERITY_HYPOPEDES_SIG_MAX,
                sig_beta=param._SEVERITY_HYPOPEDES_SIG_BETA,
                gom_max=param._SEVERITY_HYPOPEDES_GOM_MAX,
                gom_beta=param._SEVERITY_HYPOPEDES_GOM_BETA)

            hpcol_risk = rfnc.collisionRisk(
                col_severity=hpcol_severity,
                col_rate=hpcol_rate)

            total_risk += hpcol_risk
            total_eventRate += hpcol_rate
            hypoPedes.setCollisionProb(hpcol_indicator)

        for hypoVeh in l_obj['hypoVehicle']:
            hvPose, hvPoly = hypoVeh.getPredictAt(timestamp_s)
            hvcol_indicator = rfnc.collisionIndicator(
                egoPose=egoPose, egoPoly=egoPoly,
                objPose=hvPose, objPoly=hvPoly)

            hvcol_rate = rfnc.collisionEventRate(
                collisionIndicator=hvcol_indicator*hypoVeh._appearRate,
                method=param._EVENT_RATE_HYPOVEH_MODEL,
                eventRate_max=param._COLLISION_HYPOVEH_RATE_MAX,
                exp_beta=param._EVENT_RATE_HYPOVEH_EXP_BETA,
                sig_beta=param._EVENT_RATE_HYPOVEH_SIG_BETA)
            if useAwarenessRate:
                hvcol_rate *= hypoVeh._interactRate

            hvcol_severity = rfnc.collisionSeverityHypoVeh(
                ego_vx=egoPose.vdy.vx_ms, obj_vx=hvPose.vdy.vx_ms,
                method=param._SEVERITY_HYPOVEH_MODEL,
                quad_weight=param._SEVERITY_QUAD_WEIGHT,
                min_weight=param._SEVERITY_HYPOVEH_MIN_WEIGHT,
                sig_max=param._SEVERITY_HYPOVEH_SIG_MAX,
                sig_avg_vx=param._SEVERITY_HYPOVEH_AVG_VX,
                sig_beta=param._SEVERITY_HYPOVEH_SIG_B)

            hvcol_risk = rfnc.collisionRisk(
                col_severity=hvcol_severity,
                col_rate=hvcol_rate)

            total_risk += hvcol_risk
            total_eventRate += hvcol_rate
            hypoVeh.setCollisionProb(hvcol_indicator)

        self._p_eventRate.update({timestamp_s: total_eventRate})
        return total_risk

    def _survivalRate(self, timestamp_s: float, dT=param._PREDICT_STEP):
        """
        Compute survival function up to given timestamp_s
        """
        total_eventRate = self._escapeRate(param._ESCAPE_RATE)
        total_eventRate += sum(list((self._p_eventRate[k])
                               for k in self._p_eventRate if k <= timestamp_s))
        s = np.exp(-total_eventRate*dT)
        return s

    def _utilityCost(self, timestamp_s: float, u_in: float,
                     wV=param._C_CRUISE, vx=param._C_V_CRUISE,
                     wA=param._C_COMFORT, wJ=param._C_JERK):
        """
        Compute current utility cost at given predict timestamp
        """
        p_pose = self._p_pose[timestamp_s]
        utCost = 0
        utCost += wV * (p_pose.vdy.vx_ms-vx)**2
        utCost += wA * (u_in**2)
        utCost += wJ * ((u_in - self._u)**2)
        return utCost

    def _computeCost(self, timestamp_s: float, u_in: float):
        """
        Compute total cost at given timestamp
        """
        utilCost = self._utilityCost(
            timestamp_s=timestamp_s, u_in=u_in,
            wV=param._C_CRUISE, vx=param._C_V_CRUISE,
            wA=param._C_COMFORT, wJ=param._C_JERK)
        riskCost = self._riskCost(
            timestamp_s, u_in,
            useAwarenessRate=param._ENABLE_AWARENESS_RATE,
            useFOV=param._ENABLE_FOV_AWARE,
            minCollisionBrakeVehicle=param._MIN_COL_BRAKE_VEHICLE,
            minCollisionBrakePedes=param._MIN_COL_BRAKE_PEDESTRIAN)
        cost = utilCost + riskCost
        return cost

    def _computeTotalCost(self, u_in: float, dT=param._PREDICT_STEP,
                          predictTime=param._PREDICT_TIME):
        """
        Predict and compute total cost of prediction
        """
        cost = 0
        self._brake = False
        self._minColValue = 0
        self._p_eventRate = {}
        self._predict(u_in, dT=dT, predictTime=predictTime)

        for k in self._p_pose:
            dCost = self._computeCost(k, u_in)
            s = self._survivalRate(k, dT=dT)
            cost += dCost * s
        cost = cost * s * dT
        return cost

    def _computeTTB(self, aBrake=param._A_MAX_BRAKE, delay=param._T_BRAKE):
        ego_vx = self.getCurrentPose().vdy.vx_ms
        self._TTB = abs(ego_vx / aBrake) + delay

    def _move(self, dT=param._dT):
        lastPose = self.getCurrentPose()
        nextTimestamp_s = round(lastPose.timestamp_s + dT, 2)

        if self._p_u is not None:
            nextPose = pfnc.updatePose(
                lastPose=lastPose, u_in=self._p_u, dT=dT)
            self._u = self._p_u
        else:
            nextPose = pfnc.updatePose(
                lastPose=lastPose, u_in=self._u, dT=dT)
        self._currentPose = nextPose
        self._l_pose.update({nextTimestamp_s: nextPose})
        self._l_u.update({nextTimestamp_s: self._u})

        if self.getCurrentPose().vdy.vx_ms == 0 and self._emergencyState:
            self._toStopState()
            self._u = 0

    def optimizeState(self, dT=param._dT, predictStep=param._PREDICT_STEP,
                      predictTime=param._PREDICT_TIME):
        self._searchEnvironment()
        self._computeTTB(aBrake=param._A_MAX_BRAKE, delay=param._T_BRAKE)
        val = 0
        self._l_opt = []

        if self._stopState:
            val = optimize.minimize_scalar(
                lambda x: self._computeTotalCost(
                    u_in=x, dT=predictStep, predictTime=predictTime),
                bounds=(0, param._J_MAX), method='bounded',
                options={"maxiter": 5}
                ).x
            self._computeTotalCost(
                u_in=val, dT=predictStep, predictTime=predictTime)
            if self._brake or self._minColValue > 0.7:
                self._p_u = 0
                self._u = 0
            else:
                self._toDefaultState()
                self._p_u = self._u + (val - self._u) * dT / predictStep
            self._move()
            return

        if self._defaultState:
            lowBound = max(self._u - param._J_MAX, param._A_MIN)
            upBound = min(self._u + param._J_MAX, param._A_MAX)
            if lowBound >= upBound:
                lowBound = param._A_MIN
                upBound = lowBound + param._J_MAX
            val = optimize.minimize_scalar(
                lambda x: self._computeTotalCost(
                    u_in=x, dT=predictStep, predictTime=predictTime),
                bounds=(lowBound, upBound), method='bounded',
                options={"maxiter": 5}
                ).x
            self._computeTotalCost(
                u_in=val, dT=predictStep, predictTime=predictTime)
            if self._brake:
                self._toEmergencyState()
            self._p_u = self._u + (val - self._u) * dT / predictStep
            self._move()
            return

        if self._emergencyState:
            val = optimize.minimize_scalar(
                lambda x: self._computeTotalCost(
                    u_in=x, dT=predictStep, predictTime=predictTime),
                bounds=(param._A_MAX_BRAKE, param._A_MIN), method='bounded',
                options={"maxiter": 5}
            ).x
            self._p_u = self._u + (val - self._u) * dT / predictStep
            self._move()
            return

    def _toStopState(self):
        self._stopState = True
        self._defaultState = False
        self._emergencyState = False

    def _toDefaultState(self):
        self._stopState = False
        self._defaultState = True
        self._emergencyState = False

    def _toEmergencyState(self):
        self._stopState = False
        self._defaultState = False
        self._emergencyState = True

    # ------------------- System function ---------------------

    def restart(self):
        t = min(self._l_pose)
        firstPose = self._l_pose[t]
        self._l_pose = {t: firstPose}
        self._u = self._l_u[t]
        self._l_u = {t: self._l_u[t]}
        self._currentPose = firstPose

    def exportPredictState(self):
        l_p = []
        for p_pose in self._p_pose:
            exportP = {
                'pos': [p_pose.x_m, p_pose.y_m],
                'cov': p_pose.covUtm,
                'poly': pfnc.rectangle(p_pose, self._length, self._width),
            }
            l_p.append(exportP)
        return l_p

    # ------------------- Export function ---------------------

    def plotDynamicDistance(self):
        fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(14, 6))
        self._plotVelocity(ax=ax[0], xDistance=True)
        self._plotAcceleration(ax=ax[1], xDistance=True)
        plt.show()

    def plotDynamic(self):
        fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(14, 6))
        self._plotVelocity(ax=ax[0])
        self._plotAcceleration(ax=ax[1])
        plt.show()

    def saveDynamic(self, path, fileName):
        l_vdy = np.empty((0, 3))
        for t, pose in self._l_pose.items():
            u = self._l_u[t]
            l_vdy = np.append(
                l_vdy, np.array([[t, pose.vdy.vx_ms, u]]), axis=0)
        np.savetxt(path + fileName, l_vdy, fmt='%1.2f')

    def saveDynamicDistance(self, path, fileName):
        l_vdy = np.empty((0, 3))
        startPose = self._l_pose[min(self._l_pose)]
        startPos = np.array([[startPose.x_m, startPose.y_m]])
        for t, pose in self._l_pose.items():
            u = self._l_u[t]
            pos = np.array([[pose.x_m, pose.y_m]])
            d = np.linalg.norm(pos - startPos)
            l_vdy = np.append(
                l_vdy, np.array([[d, pose.vdy.vx_ms, u]]), axis=0)
        np.savetxt(path + fileName, l_vdy)

    def saveRisk(self, path, fileName):
        return

    def _plotVelocity(self, ax=plt, xDistance=False):
        l_vdy = np.empty((0, 2))
        if xDistance:
            startPose = self._l_pose[min(self._l_pose)]
            startPos = np.array([[startPose.x_m, startPose.y_m]])
            for t, pose in self._l_pose.items():
                pos = np.array([[pose.x_m, pose.y_m]])
                d = np.linalg.norm(pos - startPos)
                l_vdy = np.append(
                    l_vdy, np.array([[d, pose.vdy.vx_ms]]), axis=0)
            ax.plot(l_vdy[:, 0], l_vdy[:, 1], 'b-', label='velocity')
            ax.set_xlabel("Travel distance [s]")
            ax.set_ylabel("Velocity [m/s]")
            ax.set_ylim(0, 15)
        else:
            for t, pose in self._l_pose.items():
                l_vdy = np.append(
                    l_vdy, np.array([[t, pose.vdy.vx_ms]]), axis=0)

            ax.plot(l_vdy[:, 0], l_vdy[:, 1], 'b-', label='velocity')
            ax.set_xlabel("Time [s]")
            ax.set_ylabel("Velocity [m/s]")
            ax.set_ylim(0, 15)
        ax.legend()

    def _plotAcceleration(self, ax=plt, xDistance=False):
        l_u = sorted(self._l_u.items())
        x, y = zip(*l_u)
        if xDistance:
            l_acc = np.empty((0, 2))
            startPose = self._l_pose[min(self._l_pose)]
            startPos = np.array([[startPose.x_m, startPose.y_m]])
            for t, pose in self._l_pose.items():
                pos = np.array([[pose.x_m, pose.y_m]])
                d = np.linalg.norm(pos - startPos)
                l_acc = np.append(
                    l_acc, np.array([[d, self._l_u[t]]]), axis=0)
            ax.plot(l_acc[:, 0], l_acc[:, 1], 'r-', label='acceleration')
            ax.set_xlabel("Travel distance [s]")
            ax.set_ylabel("Acceleration [$m/s_2$]")
            ax.set_ylim(-6, 3)
        else:
            ax.plot(x, y, 'r-', label='acceleration')
            ax.set_xlabel("Time [s]")
            ax.set_ylabel("Acceleration [$m/s_2$]")
            ax.set_ylim(-6, 3)
        ax.legend()

    def plotPassedCost(self):
        l_cost = self._getPassedCost()
        survival = np.exp(- np.cumsum(l_cost[:, 2]) * param._PREDICT_STEP)
        fig, ax = plt.subplots(nrows=1, ncols=3, figsize=(21, 6))
        ax[0].plot(l_cost[:, 0], l_cost[:, 1], 'k-', label='collision risk')
        ax[1].plot(l_cost[:, 0], l_cost[:, 2], 'b-', label='collision rate')
        ax[2].set_ylim(-0.2, 1.2)
        ax[2].plot(l_cost[:, 0], survival, 'r-', label='survival rate')
        for a in ax:
            a.legend()
        plt.show()

    def _getPassedCost(self):
        l_cost = np.empty((0, 3))

        """
        Compute cost for all passed trajectory to recorded environment
        Return:
            l_cost(timestamp: cost)
        """
        for k in self._l_pose:
            total_risk = 0
            total_rate = 0

            egoPose = self._l_pose[k]
            egoPoly = self.getPoly(k)
            l_obj = self._env.updateAt(egoPose, k, self._l_u[k],
                                       radius=param._SCAN_RADIUS,
                                       hypothesis=param._ENABLE_HYPOTHESIS)

            for veh in l_obj['vehicle']:
                vehPose = veh.getPoseAt(k)
                vehPoly = veh.getPoly(k)
                vcol_indicator = rfnc.collisionIndicator(
                    egoPose=egoPose, egoPoly=egoPoly,
                    objPose=vehPose, objPoly=vehPoly)

                vcol_rate = rfnc.collisionEventRate(
                    collisionIndicator=vcol_indicator)

                vcol_severity = rfnc.collisionEventSeverity(
                    ego_vx=egoPose.vdy.vx_ms, obj_vx=vehPose.vdy.vx_ms)

                vcol_risk = rfnc.collisionRisk(
                    col_severity=vcol_severity,
                    col_rate=vcol_rate)

                total_rate += vcol_rate
                total_risk += vcol_risk

            for veh in l_obj['pedestrian']:
                vehPose = veh.getPoseAt(k)
                vehPoly = veh.getPoly(k)
                vcol_indicator = rfnc.collisionIndicator(
                    egoPose=egoPose, egoPoly=egoPoly,
                    objPose=vehPose, objPoly=vehPoly)

                vcol_rate = rfnc.collisionEventRate(
                    collisionIndicator=vcol_indicator)

                vcol_severity = rfnc.collisionEventSeverity(
                    ego_vx=egoPose.vdy.vx_ms, obj_vx=vehPose.vdy.vx_ms)

                vcol_risk = rfnc.collisionRisk(
                    col_severity=vcol_severity,
                    col_rate=vcol_rate)

                total_rate += vcol_rate
                total_risk += vcol_risk

            l_cost = np.append(
                l_cost, np.array([[k, total_risk, total_rate]]), axis=0)
        return l_cost

    # ------------------- Backup function ---------------------
    def optimize(self, dT=param._dT, predictStep=param._PREDICT_STEP):
            val = 0
            ego_vx = self.getCurrentPose().vdy.vx_ms
            # search environment
            self._searchEnvironment()

            # handle stop case
            if ego_vx == 0:
                self._u = 0
                lowBound = 0
                upBound = param._J_MAX
                val = optimize.minimize_scalar(
                    lambda x: self._computeTotalCost(
                        u_in=x, dT=predictStep),
                    bounds=(lowBound, upBound), method='bounded',
                    options={"maxiter": 5}
                    ).x
            else:
                lowBound = max(self._u - param._J_MAX, param._A_MIN)
                upBound = min(self._u + param._J_MAX, param._A_MAX)
                if lowBound >= upBound:
                    lowBound = param._A_MIN
                    upBound = lowBound + param._J_MAX
                val = optimize.minimize_scalar(
                    lambda x: self._computeTotalCost(
                        u_in=x, dT=predictStep),
                    bounds=(lowBound, upBound), method='bounded',
                    options={"maxiter": 5}
                    ).x

            self._predict(u_in=val)

            # total_eventRate = sum(
            #     list((self._p_eventRate[k]) for k in self._p_eventRate))

            # check if critical event occur to perform emergency brake
            max_eventRate = max(self._p_eventRate.values())
            if max_eventRate > param._COLLISION_RATE_BRAKE_MIN and self.getCurrentPose().vdy.vx_ms > 0:
                val = optimize.minimize_scalar(
                    lambda x: self._computeTotalCost(
                        u_in=x, dT=predictStep),
                    bounds=(param._A_MAX_BRAKE, param._A_MIN), method='bounded',
                    options={"maxiter": 5}
                ).x
            self._predict(u_in=val)
            self._p_u = self._u + (val - self._u) * dT / predictStep
            self._move()

from matplotlib.patches import Ellipse, Polygon
import matplotlib.pyplot as plt
from scipy import optimize

import numpy as np
import time

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

        self._p_u = None
        self._p_pose = {}
        self._p_eventRate = {}

        self._l_u = {startPose.timestamp_s: u_in}
        self._l_pose = {startPose.timestamp_s: startPose}
        self._currentPose = startPose
        self._u = u_in

        self._env = env
        self._fov = None
        self._l_currentObject = {}

    def getCurrentPose(self):
        return self._currentPose

    def getCurrentTimestamp(self):
        return self._currentPose.timestamp_s

    def getPoseAt(self, timestamp_s):
        if timestamp_s in self._l_pose:
            return self._l_pose[timestamp_s]
        else:
            print("Ego: No pose at: ", timestamp_s)
            return None

    def getPoly(self, timestamp_s):
        """
        Return the bounding polygon of vehicle
        """
        if timestamp_s in self._l_pose:
            pose = self._l_pose[timestamp_s]
            return pfnc.rectangle(pose.x_m, pose.y_m, pose.yaw_rad,
                                  self.length, self.width)
        else:
            print("Ego: No state at: ", timestamp_s)
            return None

    def getPredictPoly(self, timestamp_s):
        if timestamp_s in self._p_pose:
            pose = self._p_pose[timestamp_s]
            return pfnc.rectangle(pose.x_m, pose.y_m, pose.yaw_rad,
                                  self.length, self.width)
        else:
            print("Ego: No predict state at: ", timestamp_s)
            return None

    # ------------------- Opt function ---------------------

    def _searchEnvironment(self):
        """
        Scan environment at current state
        """
        currentPose = self.getCurrentPose()
        self._l_currentObject, self._fov = self._env.update(
            pose=currentPose,
            from_timestamp=currentPose.timestamp_s
            )

    def _predict(self, u_in, predictTime=param._PREDICT_TIME):
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
            nextTimestamp_s=lastPose.timestamp_s + predictTime
            )
        self._p_u = u_in

    def _escapeRate(self):
        """
        Return escape rate
        """
        return param._ESCAPE_RATE

    def _riskCost(self, timestamp_s):
        """
        Compute collision risk & event rate at given predict timestamp
        """
        timestamp_s = round(timestamp_s, 3)
        egoPose = self._p_pose[timestamp_s]
        egoPoly = self.getPredictPoly(timestamp_s)
        l_obj = self._l_currentObject
        total_risk = 0
        total_eventRate = 0

        for veh in l_obj['vehicle']:
            vehPose, vehPoly = veh.getPredictAt(timestamp_s)
            vcol_risk, vcol_rate, vcol_ind = rfnc.collisionRisk(
                egoPose=egoPose,
                egoPoly=egoPoly,
                objPose=vehPose,
                objPoly=vehPoly
                )
            total_risk += vcol_risk
            total_eventRate += vcol_rate

        for sobj in l_obj['staticObject']:
            dMerge, MP, dVis, randVertex = pfnc.distanceToMergePoint(
                pose=egoPose,
                poly=sobj._poly
                )
            if MP is not None:
                indicator, rate, risk = rfnc.unseenEventRisk(
                    d2MP=dMerge,
                    ego_vx=egoPose.vdy.vx_ms,
                    ego_acc=self._p_u,
                    dVis=dVis
                )
                total_risk += risk
                total_eventRate += rate

        for pedes in l_obj['pedestrian']:
            pedesPose, pedesPoly = pedes.getPredictAt(timestamp_s)
            pcol_risk, pcol_rate, pcol_ind = rfnc.collisionRisk(
                egoPose=egoPose,
                egoPoly=egoPoly,
                objPose=pedesPose,
                objPoly=pedesPoly
                )
            total_risk += pcol_risk
            total_eventRate += pcol_rate

        self._p_eventRate.update({timestamp_s: total_eventRate})
        return total_risk

    def _survivalRate(self, timestamp_s, dT=param._dT):
        """
        Compute survival function up to given timestamp_s
        """
        total_eventRate = self._escapeRate()
        total_eventRate += sum(list((self._p_eventRate[k])
                               for k in self._p_eventRate if k <= timestamp_s))
        s = np.exp(-total_eventRate*dT)
        return s

    def _utilityCost(self, timestamp_s, u_in):
        """
        Compute current utility cost at given predict timestamp
        """
        p_pose = self._p_pose[timestamp_s]
        utCost = 0
        utCost += param._C_CRUISE * ((p_pose.vdy.vx_ms - param._C_V_CRUISE)**2)
        utCost += param._C_COMFORT * (u_in**2)
        utCost += param._C_JERK * ((u_in - self._u)**2)
        return utCost

    def _computeCost(self, timestamp_s, u_in):
        """
        Compute total cost at given timestamp
        """
        utilCost = self._utilityCost(timestamp_s, u_in)
        riskCost = self._riskCost(timestamp_s)
        cost = utilCost + riskCost
        return cost

    def _computeTotalCost(self, u_in, dT=param._dT):
        """
        Predict and compute total cost of prediction
        """
        self._p_eventRate = {}
        self._predict(u_in)
        cost = 0
        for k in self._p_pose:
            dCost = self._computeCost(k, u_in)
            s = self._survivalRate(k)
            cost += dCost * s
        cost = cost * s * dT
        return cost

    def _move(self, dT=param._dT):
        lastPose = self.getCurrentPose()
        nextTimestamp_s = round(lastPose.timestamp_s + dT, 3)
        if self._p_u is not None:
            nextPose = pfnc.updatePose(
                lastPose=lastPose,
                u_in=self._p_u,
                dT=dT
                )
            self._u = self._p_u
        else:
            nextPose = pfnc.updatePose(
                lastPose=lastPose,
                u_in=self._u,
                dT=dT
                )
        self._currentPose = nextPose
        self._l_pose.update({nextTimestamp_s: nextPose})
        self._l_u.update({nextTimestamp_s: self._u})

    def optimize(self):
        val = 0
        lowBound = max(self._u - 0.5, param._A_MIN)
        upBound = min(self._u + 0.5, param._A_MAX)

        # search environment
        self._searchEnvironment()

        # handle stop case
        if self.getCurrentPose().vdy.vx_ms < 0.1:
            val = optimize.minimize_scalar(
                lambda x: self._computeTotalCost(u_in=x, dT=param._dT),
                bounds=(0, param._A_MAX), method='bounded',
                options={"maxiter": 3}
                ).x
        else:
            val = optimize.minimize_scalar(
                lambda x: self._computeTotalCost(u_in=x, dT=param._dT),
                bounds=(self._u - 0.5, self._u + 0.5), method='bounded',
                options={"maxiter": 3}
                ).x

        # check if critical event occur
        total_eventRate = sum(
            list((self._p_eventRate[k]) for k in self._p_eventRate))
        # max_eventRate = max(self._p_eventRate.values())
        # if max_eventRate > 1:
        if total_eventRate > 2:
            val = optimize.minimize_scalar(
                lambda x: self._computeTotalCost(u_in=x, dT=param._dT),
                bounds=(-8, param._A_MIN), method='bounded',
                options={"maxiter": 5}
            ).x

        self._p_u = val
        self._move()

    # ------------------- Test function ---------------------

    def plotPose(self, maxTimestamp_s, ax=plt):
        for timestamp_s, pose in self._l_pose.items():
            if timestamp_s <= maxTimestamp_s:
                plt.scatter(pose.x_m, pose.y_m, s=1, color='r')
                cov = pose.covLatLong
                ellipse = Ellipse(
                    [pose.x_m, pose.y_m],
                    width=np.sqrt(cov[0, 0])*2,
                    height=np.sqrt(cov[1, 1])*2,
                    angle=np.degrees(pose.yaw_rad),
                    facecolor='mistyrose',
                    edgecolor='red',
                    alpha=0.7
                    )
                ax.add_patch(ellipse)
                if timestamp_s == maxTimestamp_s:
                    poly = self.getPoly(timestamp_s)
                    poly = Polygon(
                        poly, facecolor='yellow',
                        edgecolor='gold', alpha=0.7, label='ego vehicle'
                    )
                    ax.add_patch(poly)
                    l_obj = self._env.update(
                                x_m=pose.x_m,
                                y_m=pose.y_m,
                                timestamp_s=pose.timestamp_s,
                                from_timestamp=pose.timestamp_s
                                )
                    polys = []
                    for obj in l_obj['staticObject']:
                        objPoly = obj._poly
                        d2MP, MP, dVis, randVertex = pfnc.distanceToMergePoint(
                            pose=pose,
                            poly=objPoly
                            )
                        if MP is not None:
                            plt.scatter(randVertex[0], randVertex[1], color='r')
                            plt.scatter(MP[0], MP[1])
                            indicator, rate, risk = rfnc.unseenEventRisk(
                                d2MP=d2MP,
                                ego_vx=pose.vdy.vx_ms,
                                ego_acc=self._l_u[timestamp_s],
                                dVis=dVis
                            )
                        polys.append(objPoly)
                    fov = pfnc.FOV(pose, polys, np.pi/6, 25)
                    for f in fov:
                        plt.plot([pose.x_m, f[0]], [pose.y_m, f[1]], color='r', alpha=0.3, linewidth=0.5)
            elif timestamp_s < maxTimestamp_s + param._PREDICT_TIME:
                plt.scatter(pose.x_m, pose.y_m, s=1, color='m')
                cov = pose.covLatLong
                ellipse = Ellipse(
                    [pose.x_m, pose.y_m],
                    width=np.sqrt(cov[0, 0])*2,
                    height=np.sqrt(cov[1, 1])*2,
                    angle=np.degrees(pose.yaw_rad),
                    facecolor='wheat',
                    edgecolor='orange',
                    alpha=0.4
                    )
                ax.add_patch(ellipse)

    def plotDynamic(self):
        fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(14, 6))
        self._plotVelocity(ax=ax[0])
        self._plotAcceleration(ax=ax[1])

    def _plotVelocity(self, ax=plt):
        l_vdy = np.empty((0, 2))
        for t, pose in self._l_pose.items():
            l_vdy = np.append(
                l_vdy, np.array([[t, pose.vdy.vx_ms]]), axis=0)

        ax.plot(l_vdy[:, 0], l_vdy[:, 1], 'b-', label='velocity')
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Velocity [m/s]")
        ax.legend()

    def _plotAcceleration(self, ax=plt):
        l_u = sorted(self._l_u.items())
        x, y = zip(*l_u)
        ax.plot(x, y, 'r-', label='acceleration')
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Acceleration [$m/s_2$]")
        ax.legend()

    def plotPassedCost(self):
        l_cost = self._getPassedCost()
        survival = np.exp(- np.cumsum(l_cost[:, 2]) * param._dT)
        fig, ax = plt.subplots(nrows=1, ncols=3, figsize=(21, 6))
        ax[0].plot(l_cost[:, 0], l_cost[:, 1], 'k-', label='collision risk')
        ax[1].plot(l_cost[:, 0], l_cost[:, 2], 'b-', label='collision rate')
        ax[2].set_ylim(-0.2, 1.2)
        ax[2].plot(l_cost[:, 0], survival, 'r-', label='survival rate')
        for a in ax:
            a.legend()

    def _getPassedCost(self):
        l_cost = np.empty((0, 3))

        """
        Compute cost for all passed trajectory to recorded environment
        Return:
            l_cost(timestamp: cost)
        """
        for k in self._l_pose:
            col_risk = 0
            col_rate = 0
            egoPose = self._l_pose[k]
            egoPoly = self.getPoly(k)
            l_obj = self._env.updateAt(x_m=egoPose.x_m,
                                       y_m=egoPose.y_m,
                                       timestamp_s=egoPose.timestamp_s)
            for obj in l_obj['vehicle']:
                objPose = obj.getPoseAt(k)
                objPoly = obj.getPoly(k)
                col_risk1, col_rate1, col_ind1 = rfnc.collisionRisk(
                    egoPose=egoPose,
                    egoPoly=egoPoly,
                    objPose=objPose,
                    objPoly=objPoly
                )
                col_risk += col_risk1
                col_rate += col_rate1
            for obj in l_obj['staticObject']:
                objPoly = obj._poly
                # define unexpected risk here and add rate and risk
                dMerge, MP, dVis, randVertex = pfnc.distanceToMergePoint(
                    pose=egoPose,
                    poly=objPoly
                    )
                if MP is not None:
                    indicator, rate, risk = rfnc.unseenEventRisk(
                        d2MP=dMerge,
                        ego_vx=egoPose.vdy.vx_ms,
                        ego_acc=self._l_u[k],
                        dVis=dVis
                    )
                    col_risk += risk
                    col_rate += rate

            l_cost = np.append(
                l_cost, np.array([[k, col_risk, col_rate]]), axis=0)
        return l_cost

    def _unseenObjectCost(self):
        nrObj = len(self._env._l_staticObject)
        l_cost_list = {}

        for i in range(0, nrObj):
            l_cost = np.empty((0, 4))
            l_cost_list.update({i: l_cost})

        for k in self._l_pose:
            egoPose = self._l_pose[k]
            l_obj = self._env.updateAt(x_m=egoPose.x_m,
                                       y_m=egoPose.y_m,
                                       timestamp_s=egoPose.timestamp_s)

            for obj in l_obj['staticObject']:
                indicator, rate, risk = 0, 0, 0
                # define unexpected risk here and add rate and risk
                dMerge, MP, dVis, randVertex = pfnc.distanceToMergePoint(
                    pose=egoPose,
                    poly=obj._poly
                    )
                if MP is not None:
                    indicator, rate, risk = rfnc.unseenEventRisk(
                        d2MP=dMerge,
                        ego_vx=egoPose.vdy.vx_ms,
                        ego_acc=self._l_u[k],
                        dVis=dVis
                    )

                l_cost_list[obj._idx-1] = np.append(
                    l_cost_list[obj._idx-1], np.array([[k, indicator, rate, risk]]), axis=0)

        fig, ax = plt.subplots(nrows=1, ncols=3, figsize=(21, 6))

        ax[0].set_title("Probability of collision with unseen pedestrian")
        ax[0].set_xlabel("Time [s]")
        ax[0].set_ylabel("Probability")
        ax[0].set_ylim(0, 1)

        ax[1].set_title("Event rate of collision with unseen pedestrian")
        ax[1].set_xlabel("Time [s]")
        ax[1].set_ylabel("Event rate [event/s]")
        ax[1].set_ylim(0, 1)

        ax[2].set_title("Risk of collision with unseen pedestrian")
        ax[2].set_xlabel("Time [s]")
        ax[2].set_ylabel("Risk")

        for (i, l_cost) in l_cost_list.items():

            ax[0].plot(l_cost[:, 0], l_cost[:, 1],
                       label='object {:}'.format(i+1))

            ax[1].plot(l_cost[:, 0], l_cost[:, 2],
                       label='object {:}'.format(i+1))

            ax[2].plot(l_cost[:, 0], l_cost[:, 3],
                       label='object {:}'.format(i+1))

        for a in ax:
            a.legend()

        return l_cost_list

    def _unseenObjectCost_unseenRate(self):
        nrObj = len(self._env._l_staticObject)
        l_cost_list = {}
        unseenRate = [0.1, 0.5, 1, 2]
        for i in unseenRate:
            l_cost = np.empty((0, 5))
            l_cost_list.update({i: l_cost})

        for k in self._l_pose:
            egoPose = self._l_pose[k]
            l_obj = self._env.updateAt(x_m=egoPose.x_m,
                                       y_m=egoPose.y_m,
                                       timestamp_s=egoPose.timestamp_s)

            for obj in l_obj['staticObject']:
                for i in unseenRate:
                    indicator, rate, risk = 0, 0, 0
                    # define unexpected risk here and add rate and risk
                    dMerge, MP, dVis, randVertex = pfnc.distanceToMergePoint(
                        pose=egoPose,
                        poly=obj._poly
                        )
                    if MP is not None:
                        indicator, rate, risk = rfnc.unseenEventRisk(
                            d2MP=dMerge,
                            ego_vx=egoPose.vdy.vx_ms,
                            ego_acc=self._l_u[k],
                            dVis=dVis,
                            objectAppearRate=i
                        )
                    else:
                        dMerge = 0
                    l_cost_list[i] = np.append(
                        l_cost_list[i], np.array([[k, dMerge, indicator, rate, risk]]), axis=0)                  

        fig, ax = plt.subplots(nrows=1, ncols=3, figsize=(21, 6))

        ax[0].set_title("Probability of collision with unseen pedestrian")
        ax[0].set_xlabel("Distane to merge point [m]")
        ax[0].set_ylabel("Probability")
        ax[0].set_ylim(0, 1)

        ax[1].set_title("Event rate of collision with unseen pedestrian")
        ax[1].set_xlabel("Distane to merge point [m]")
        ax[1].set_ylabel("Event rate [event/s]")
        ax[1].set_ylim(0, 1)

        ax[2].set_title("Risk of collision with unseen pedestrian")
        ax[2].set_xlabel("Distane to merge point [m]")
        ax[2].set_ylabel("Risk")

        for (i, l_cost) in l_cost_list.items():

            ax[0].plot(l_cost[:, 1], l_cost[:, 2],
                       label='unseen rate = {:}'.format(i))

            ax[1].plot(l_cost[:, 1], l_cost[:, 3],
                       label='unseen rate = {:}'.format(i))

            ax[2].plot(l_cost[:, 1], l_cost[:, 4],
                       label='unseen rate = {:}'.format(i))

        for a in ax:
            a.legend()

        ax[0].set_xlim(25, -2)
        ax[1].set_xlim(25, -2)
        ax[2].set_xlim(25, -2)

        fig, ax = plt.subplots(nrows=1, ncols=3, figsize=(21, 6))

        ax[0].set_title("Probability of collision with unseen pedestrian")
        ax[0].set_xlabel("Time [s]")
        ax[0].set_ylabel("Probability")
        ax[0].set_ylim(0, 1)

        ax[1].set_title("Event rate of collision with unseen pedestrian")
        ax[1].set_xlabel("Time [s]")
        ax[1].set_ylabel("Event rate [event/s]")
        ax[1].set_ylim(0, 1)

        ax[2].set_title("Risk of collision with unseen pedestrian")
        ax[2].set_xlabel("Time [s]")
        ax[2].set_ylabel("Risk")

        for (i, l_cost) in l_cost_list.items():

            ax[0].plot(l_cost[:, 0], l_cost[:, 2],
                       label='unseen rate = {:}'.format(i))

            ax[1].plot(l_cost[:, 0], l_cost[:, 3],
                       label='unseen rate = {:}'.format(i))

            ax[2].plot(l_cost[:, 0], l_cost[:, 4],
                       label='unseen rate = {:}'.format(i))

        for a in ax:
            a.legend()

        return l_cost_list

    def _unseenObjectCost_velocity(self):
        l_cost_list = {}
        velocity = [4, 6, 8, 10]
        for i in velocity:
            l_cost = np.empty((0, 6))
            l_cost_list.update({i: l_cost})

        for k in self._l_pose:
            egoPose = self._l_pose[k]
            l_obj = self._env.updateAt(x_m=egoPose.x_m,
                                       y_m=egoPose.y_m,
                                       timestamp_s=egoPose.timestamp_s)

            for obj in l_obj['staticObject']:
                for i in velocity:
                    indicator, rate, risk = 0, 0, 0
                    # define unexpected risk here and add rate and risk
                    dMerge, MP, dVis, randVertex = pfnc.distanceToMergePoint(
                        pose=egoPose,
                        poly=obj._poly
                        )
                    if MP is not None:
                        indicator, rate, risk = rfnc.unseenEventRisk(
                            d2MP=dMerge,
                            ego_vx=i,
                            ego_acc=self._l_u[k],
                            dVis=dVis
                        )
                    else:
                        dMerge, dVis = 0, 0
                    l_cost_list[i] = np.append(
                        l_cost_list[i], np.array([[k, dMerge, dVis, indicator, rate, risk]]), axis=0)                  

        fig, ax = plt.subplots(nrows=1, ncols=3, figsize=(21, 6))

        ax[0].set_title("Probability of collision with unseen pedestrian")
        ax[0].set_xlabel("Distane to merge point [m]")
        ax[0].set_ylabel("Probability")
        ax[0].set_ylim(0, 1)

        ax[1].set_title("Event rate of collision with unseen pedestrian")
        ax[1].set_xlabel("Distane to merge point [m]")
        ax[1].set_ylabel("Event rate [event/s]")
        ax[1].set_ylim(0, 1)

        ax[2].set_title("Risk of collision with unseen pedestrian")
        ax[2].set_xlabel("Distane to merge point [m]")
        ax[2].set_ylabel("Risk")

        for (i, l_cost) in l_cost_list.items():

            ax[0].plot(l_cost[:, 1], l_cost[:, 3],
                       label='$v_x$ = {:} [m/s]'.format(i))

            ax[1].plot(l_cost[:, 1], l_cost[:, 4],
                       label='$v_x$ = {:} [m/s]'.format(i))

            ax[2].plot(l_cost[:, 1], l_cost[:, 5],
                       label='$v_x$ = {:} [m/s]'.format(i))

        for a in ax:
            a.legend()

        ax[0].set_xlim(25, -2)
        ax[1].set_xlim(25, -2)
        ax[2].set_xlim(25, -2)

        fig, ax = plt.subplots(nrows=1, ncols=3, figsize=(21, 6))

        ax[0].set_title("Probability of collision with unseen pedestrian")
        ax[0].set_xlabel("Time [s]")
        ax[0].set_ylabel("Probability")
        ax[0].set_ylim(0, 1)

        ax[1].set_title("Event rate of collision with unseen pedestrian")
        ax[1].set_xlabel("Time [s]")
        ax[1].set_ylabel("Event rate [event/s]")
        ax[1].set_ylim(0, 1)

        ax[2].set_title("Risk of collision with unseen pedestrian")
        ax[2].set_xlabel("Time [s]")
        ax[2].set_ylabel("Risk")

        for (i, l_cost) in l_cost_list.items():

            ax[0].plot(l_cost[:, 0], l_cost[:, 3],
                       label='$v_x$ = {:} [m/s]'.format(i))

            ax[1].plot(l_cost[:, 0], l_cost[:, 4],
                       label='$v_x$ = {:} [m/s]'.format(i))

            ax[2].plot(l_cost[:, 0], l_cost[:, 5],
                       label='$v_x$ = {:} [m/s]'.format(i))

        for a in ax:
            a.legend()

        fig, ax = plt.subplots(nrows=1, ncols=3, figsize=(21, 6))

        ax[0].set_title("Probability of collision with unseen pedestrian")
        ax[0].set_xlabel("Minimal visible range [m]")
        ax[0].set_ylabel("Probability")
        ax[0].set_ylim(0, 1)

        ax[1].set_title("Event rate of collision with unseen pedestrian")
        ax[1].set_xlabel("Minimal visible range [m]")
        ax[1].set_ylabel("Event rate [event/s]")
        ax[1].set_ylim(0, 1)

        ax[2].set_title("Risk of collision with unseen pedestrian")
        ax[2].set_xlabel("Minimal visible range [m]")
        ax[2].set_ylabel("Risk")

        for (i, l_cost) in l_cost_list.items():

            ax[0].plot(l_cost[:, 2], l_cost[:, 3],
                       label='velocity = {:}'.format(i))

            ax[1].plot(l_cost[:, 2], l_cost[:, 4],
                       label='velocity = {:}'.format(i))

            ax[2].plot(l_cost[:, 2], l_cost[:, 5],
                       label='velocity = {:}'.format(i))

        for a in ax:
            a.legend()

        return l_cost_list

    def _unseenObjectCost_dVis(self):
        l_cost_list = {}
        unseenRate = [0.1, 0.5, 1, 2]
        for i in unseenRate:
            l_cost = np.empty((0, 5))
            l_cost_list.update({i: l_cost})

        for k in self._l_pose:
            egoPose = self._l_pose[k]
            l_obj = self._env.updateAt(x_m=egoPose.x_m,
                                       y_m=egoPose.y_m,
                                       timestamp_s=egoPose.timestamp_s)

            for obj in l_obj['staticObject']:
                for i in unseenRate:
                    indicator, rate, risk = 0, 0, 0
                    # define unexpected risk here and add rate and risk
                    dMerge, MP, dVis, randVertex = pfnc.distanceToMergePoint(
                        pose=egoPose,
                        poly=obj._poly
                        )
                    if MP is not None:
                        indicator, rate, risk = rfnc.unseenEventRisk(
                            d2MP=dMerge,
                            ego_vx=egoPose.vdy.vx_ms,
                            ego_acc=self._l_u[k],
                            dVis=dVis,
                            objectAppearRate=i
                        )
                    l_cost_list[i] = np.append(
                        l_cost_list[i], np.array([[k, dVis, indicator, rate, risk]]), axis=0)                  

        fig, ax = plt.subplots(nrows=1, ncols=3, figsize=(21, 6))

        ax[0].set_title("Probability of collision with unseen pedestrian")
        ax[0].set_xlabel("Minimal visible range [m]")
        ax[0].set_ylabel("Probability")
        ax[0].set_ylim(0, 1)

        ax[1].set_title("Event rate of collision with unseen pedestrian")
        ax[1].set_xlabel("Minimal visible range [m]")
        ax[1].set_ylabel("Event rate [event/s]")
        ax[1].set_ylim(0, 1)

        ax[2].set_title("Risk of collision with unseen pedestrian")
        ax[2].set_xlabel("Minimal visible range [m]")
        ax[2].set_ylabel("Risk")

        for (i, l_cost) in l_cost_list.items():

            ax[0].plot(l_cost[:, 1], l_cost[:, 2],
                       label='unseen rate = {:}'.format(i))

            ax[1].plot(l_cost[:, 1], l_cost[:, 3],
                       label='unseen rate = {:}'.format(i))

            ax[2].plot(l_cost[:, 1], l_cost[:, 4],
                       label='unseen rate = {:}'.format(i))

        for a in ax:
            a.legend()

        fig, ax = plt.subplots(nrows=1, ncols=3, figsize=(21, 6))

        ax[0].set_title("Probability of collision with unseen pedestrian")
        ax[0].set_xlabel("Time [s]")
        ax[0].set_ylabel("Probability")
        ax[0].set_ylim(0, 1)

        ax[1].set_title("Event rate of collision with unseen pedestrian")
        ax[1].set_xlabel("Time [s]")
        ax[1].set_ylabel("Event rate [event/s]")
        ax[1].set_ylim(0, 1)

        ax[2].set_title("Risk of collision with unseen pedestrian")
        ax[2].set_xlabel("Time [s]")
        ax[2].set_ylabel("Risk")

        for (i, l_cost) in l_cost_list.items():

            ax[0].plot(l_cost[:, 0], l_cost[:, 2],
                       label='unseen rate = {:}'.format(i))

            ax[1].plot(l_cost[:, 0], l_cost[:, 3],
                       label='unseen rate = {:}'.format(i))

            ax[2].plot(l_cost[:, 0], l_cost[:, 4],
                       label='unseen rate = {:}'.format(i))

        for a in ax:
            a.legend()

        return l_cost_list

    def _getAllPassedCost(self):
        return
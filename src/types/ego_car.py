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

    def __init__(self, idx, env=None):
        self._idx = idx
        self._u = 0

        self._p_pose = {}
        self._p_u = None
        self._p_eventRate = {}

        self._l_pose = {}
        self._l_u = {}

        if env is not None:
            self._env = env

        self._is_moving = False

    def start(self, startPose, u_in):
        """
        Call this function to start the vehicle
        Args:
            startPose: start sate of vehicle
            u_in: start acceleration
        Return:
            False if vehicle is already started, otherwise True
        """
        if self._is_moving:
            return False
        else:
            self._is_moving = True
            self._u = u_in
            self._l_u[startPose.timestamp_s] = u_in
            self._l_pose[startPose.timestamp_s] = startPose
        return True

    def getCurrentPose(self):
        # debug
        if not self._l_pose:
            print("No pose. is vehicle started?")
            return
        return self._l_pose[max(self._l_pose)]

    def getCurrentTimestamp(self):
        # debug
        if not self._l_pose:
            print("No pose. is vehicle started?")
            return
        return max(self._l_pose)

    def getPoseAt(self, timestamp_s):
        if timestamp_s in self._l_pose:
            return self._l_pose[timestamp_s]
        elif timestamp_s in self._p_pose:
            return self._p_pose[timestamp_s]
        else:
            print("No pose at: ", timestamp_s)
            return None

    def getPoly(self, timestamp_s):
        """
        Return the bounding polygon of vehicle
        """

        if timestamp_s in self._p_pose:
            pose = self._p_pose[timestamp_s]
            return pfnc.rectangle(pose.x_m, pose.y_m, pose.yaw_rad,
                                  param._CAR_LENGTH, param._CAR_WIDTH)
        elif timestamp_s in self._l_pose:
            pose = self._l_pose[timestamp_s]
            return pfnc.rectangle(pose.x_m, pose.y_m, pose.yaw_rad,
                                  param._CAR_LENGTH, param._CAR_WIDTH)
        else:
            print("No state at: ", timestamp_s)
            return

    # ------------------- Opt function ---------------------

    def _searchEnvironment(self, timestamp_s):
        """
        Scan environment at given predict timestamp
        """
        p_pose = self._p_pose[timestamp_s]
        l_obj = self._env.update(
            x_m=p_pose.x_m,
            y_m=p_pose.y_m,
            timestamp_s=p_pose.timestamp_s,
            from_timestamp=self.getCurrentTimestamp()
            )
        return l_obj

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
        egoPose = self._p_pose[timestamp_s]
        egoPoly = self.getPoly(timestamp_s)
        l_obj = self._searchEnvironment(timestamp_s)
        total_risk = 0
        total_eventRate = 0
        for obj in l_obj['vehicle']:
            col_risk, col_rate, col_ind = rfnc.collisionRisk(
                egoPose=egoPose,
                egoPoly=egoPoly,
                objPose=obj.getPoseAt(timestamp_s),
                objPoly=obj.getPoly(timestamp_s)
                )
            total_risk += col_risk
            total_eventRate += col_rate
        for obj in l_obj['staticObject']:
            dMerge, MP, dVis, randVertex = pfnc.distanceToMergePoint(
                pose=egoPose,
                poly=obj._poly
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
        self._l_pose.update({nextTimestamp_s: nextPose})
        self._l_u.update({nextTimestamp_s: self._u})

    def optimize(self):
        val = 0
        lowBound = max(self._u - 0.5, param._A_MIN)
        upBound = min(self._u + 0.5, param._A_MAX)

        # handle stop case
        if self.getCurrentPose().vdy.vx_ms < 0.1:
            val = optimize.minimize_scalar(
                lambda x: self._computeTotalCost(u_in=x, dT=param._dT),
                bounds=(0, param._A_MAX), method='bounded',
                options={"maxiter": 5}
                ).x
        else:
            val = optimize.minimize_scalar(
                lambda x: self._computeTotalCost(u_in=x, dT=param._dT),
                bounds=(self._u - 0.5, self._u + 0.5), method='bounded',
                options={"maxiter": 5}
                ).x

        # check if critical event occur
        total_eventRate = sum(
            list((self._p_eventRate[k]) for k in self._p_eventRate))

        if total_eventRate > 1:
            val = optimize.minimize_scalar(
                lambda x: self._computeTotalCost(u_in=x, dT=param._dT),
                bounds=(-8, param._A_MIN), method='bounded',
                options={"maxiter": 5}
            ).x

        self._p_u = val
        self._move()

    # ------------------- Export function ---------------------

    def exportPose(self, timestamp_s):
        return

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
                        ego_acc=self._p_u,
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
                        ego_acc=self._p_u,
                        dVis=dVis
                    )

                l_cost_list[obj._idx-1] = np.append(
                    l_cost_list[obj._idx-1], np.array([[k, indicator, rate, risk]]), axis=0)

        fig, ax = plt.subplots(nrows=1, ncols=3, figsize=(21, 6))

        for (i, l_cost) in l_cost_list.items():

            ax[0].plot(l_cost[:, 0], l_cost[:, 1],
                       label='object {:}'.format(i+1))

            ax[1].plot(l_cost[:, 0], l_cost[:, 2],
                       label='object {:}'.format(i+1))

            ax[2].plot(l_cost[:, 0], l_cost[:, 3],
                       label='object {:}'.format(i+1))

        ax[0].set_title("Probability of collision with unseen pedestrian")
        ax[0].set_xlabel("Time [s]")
        ax[0].set_ylabel("Probability")

        ax[1].set_title("Event rate of collision with unseen pedestrian")
        ax[1].set_xlabel("Time [s]")
        ax[1].set_ylabel("Event rate [event/s]")

        ax[2].set_title("Risk of collision with unseen pedestrian")
        ax[2].set_xlabel("Time [s]")
        ax[2].set_ylabel("Risk")

        for a in ax:
            a.legend()

        return l_cost_list

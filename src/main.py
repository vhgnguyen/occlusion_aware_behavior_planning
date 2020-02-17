# %%
import sys
sys.path.insert(0, 'stuffs')
sys.path.insert(0, 'types')

import numpy as np
import matplotlib.pyplot as plt
import time

from pose import Pose, VehicleDynamic
from environment import Environment
from ego_car import EgoVehicle
from objects import OtherVehicle, StaticObject

import _param as param

# %%
SCENARIO_NR = 3

# -------- ENV -----------
env = Environment()
env.setupScenario(scenario=SCENARIO_NR)
# env._l_vehicle = []

# -------- EGO -----------
vehicle1 = EgoVehicle(idx=1, env=env)
init_VDY1 = VehicleDynamic(10, 0.0)
startPose1 = Pose(
    x_m=-35, y_m=-2, yaw_rad=0,
    covLatLong=np.array([[0.3, 0.0], [0.0, 0.1]]),
    vdy=VehicleDynamic(6, 0),
    timestamp_s=0)
vehicle1.start(startPose=startPose1, u_in=1)

# %%
while vehicle1.getCurrentTimestamp() < param._SIMULATION_TIME:
    vehicle1.optimize()
    vehicle1._move()

# %%
test = True
plotTimes = [3, 4, 5.4, 6, 7, 8]
if test:
    for plotTime in plotTimes:
        fig1 = plt.figure()
        ax1 = plt.gca()
        plt.grid()
        plt.xlim([-30, 20])
        plt.ylim([-10, 10])
        plt.text(-25, 7, "Time: " + str(plotTime) + " s",
                 bbox=dict(facecolor='green', alpha=0.8), fontsize=18)
        vehicle1.plotPose(maxTimestamp_s=plotTime, ax=ax1)
        env.plot(timestamp_s=plotTime, plotHistory=True, ax=ax1)
        plt.show()

# %%
vehicle1.plotDynamic()
vehicle1.plotPassedCost()
vehicle1._unseenObjectCost()

# %%
import risk_functions as rfnc
dToMerges = np.linspace(10, 0, 101)
fig = plt.figure()
for d in dToMerges:
    prob = rfnc.unseenObjectEventRate(
        dToMergePoint=d,
        ego_vx=5,
        ego_acc=0.3,
        visibleD=7)
    plt.scatter(d, prob)

# %%

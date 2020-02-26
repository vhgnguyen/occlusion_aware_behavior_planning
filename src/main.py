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
startPose1 = Pose(
    x_m=-35, y_m=0, yaw_rad=0,
    covLatLong=np.array([[0.3, 0.0], [0.0, 0.1]]),
    vdy=VehicleDynamic(6, 0),
    timestamp_s=0)
vehicle1.start(startPose=startPose1, u_in=0)

# %%
while vehicle1.getCurrentTimestamp() < param._SIMULATION_TIME:
    start = time.time()
    vehicle1.optimize()
    # vehicle1._move()
    print(time.time() - start)
# %%
test = True
plotTimes = [2, 4, 4.6, 5, 6, 8, 10, 15]
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
_ = vehicle1._unseenObjectCost()
_ = vehicle1._unseenObjectCost_unseenRate()
_ = vehicle1._unseenObjectCost_velocity()
_ = vehicle1._unseenObjectCost_dVis()

# %%


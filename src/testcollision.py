#%%
import numpy as np 
import stuffs.gaussian as gn

def rectangle(pose, length, width):
    """
    Return rectangle centered at given position
    """
    poly = np.array([[-length/2, -width/2],
                     [length/2, -width/2],
                     [length/2, width/2],
                     [-length/2, width/2]])
    return np.dot(poly, pose.getRotation()) + pose.getTranslation()


class Pose(object):

    def __init__(self, x_m, y_m, yaw_rad):
        self.x_m = x_m
        self.y_m = y_m
        self.yaw_rad = yaw_rad
        self._s = np.sin(self.yaw_rad)
        self._c = np.cos(self.yaw_rad)

    def getRotation(self):
        return np.array([[self._c, self._s], [-self._s, self._c]])

    def getTranslation(self):
        return np.array([self.x_m, self.y_m])

    def heading(self):
        return np.array([self._c, self._s])

# %%
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Polygon

pose1 = Pose(x_m=1, y_m=1, yaw_rad=0)
pose2 = Pose(x_m=4, y_m=4, yaw_rad=np.pi/2)
obj1 = rectangle(pose1, 2,1)
obj2 = rectangle(pose2, 4,1)

minK, _ = gn.minkowskiSum(-obj2, obj1)

fig, ax = plt.subplots()
poly = Polygon(
    minK, facecolor='yellow',
    edgecolor='gold', alpha=0.7
)
plt.scatter(obj1[:,0], obj1[:,1])
plt.scatter(obj2[:,0], obj2[:,1])
plt.scatter(minK[:,0], minK[:,1])
ax.add_patch(poly)
plt.show()

# %%



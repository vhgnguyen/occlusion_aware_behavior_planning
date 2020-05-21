import numpy as np
from objects import (StaticObject, Road, PedestrianCross)


def scenario(l_staticObject, l_cross, l_road):
    """
    Coordinates of objects in a scenario. Include:

        l_staticObject: list of static objects

        l_cross:    list of pedestrian crosses

        l_road: list of road layouts

    """
    # static object
    obs1 = StaticObject(
        idx=1,
        poly=np.array([[-50, -20], [-39, -20], [-39, -6],
                       [-45, -5], [-50, -5]]))
    l_staticObject.append(obs1)

    obs2 = StaticObject(
        idx=2,
        poly=np.array([[-35, -20], [-15, -20], [-15, -5],
                       [-30, -5], [-35, -6]]))
    l_staticObject.append(obs2)

    obs3 = StaticObject(
        idx=3,
        poly=np.array([[-10, -20], [3, -20], [3, -10],
                       [0, -6], [-10, -6]]))
    l_staticObject.append(obs3)

    obs4 = StaticObject(
        idx=4,
        poly=np.array([[10, -20], [20, -20], [20, -8],
                       [10, -8]]))
    l_staticObject.append(obs4)

    obs5 = StaticObject(
        idx=5,
        poly=np.array([[-60, 7], [0, 7], [6, 10],
                       [6, 20], [-60, 20]]))
    l_staticObject.append(obs5)

    # pedestrian cross
    cross1 = PedestrianCross(
        left=np.array([[5, -7], [5, 7]]),
        right=np.array([[7, -7], [7, 7]]),
        density=0.5
    )
    l_cross.append(cross1)

    # road layout
    road = Road(
        left=np.array([[-100, 4], [100, 4]]),
        right=np.array([[-100, -4], [100, -4]]),
        lane=np.array([[-100, 0], [100, 0]])
    )
    l_road.append(road)

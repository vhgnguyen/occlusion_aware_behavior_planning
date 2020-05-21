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
        poly=np.array([[-40, -20], [-10, -20], [-10, -10],
                       [-20, -5], [-40, -5]]))
    l_staticObject.append(obs1)

    obs2 = StaticObject(
        idx=2,
        poly=np.array([[8, -20], [30, -20], [30, -5], [8, -5]]))
    l_staticObject.append(obs2)

    obs3 = StaticObject(
        idx=3,
        poly=np.array([[-40, 5], [-20, 5], [-10, 8],
                       [-10, 20], [-40, 20]]))
    l_staticObject.append(obs3)

    obs4 = StaticObject(
        idx=4,
        poly=np.array([[5, 5], [10, 5], [10, 20],
                       [5, 20]]))
    l_staticObject.append(obs4)

    # pedestrian cross
    cross1 = PedestrianCross(
        left=np.array([[-7, -10], [-7, 10]]),
        right=np.array([[-4, -10], [-4, 10]]),
        density=0.8
    )
    l_cross.append(cross1)

    # road layout
    road1 = Road(
        left=np.array([[-100, 4], [-4, 4]]),
        right=np.array([[-100, -4], [-4, -4]]),
        lane=np.array([[-100, 0], [-4, 0]])
    )
    road2 = Road(
        left=np.array([[-4, 4], [-4, 100]]),
        right=np.array([[4, 4], [4, 100]]),
        lane=np.array([[0, 4], [0, 100]])
    )
    road3 = Road(
        left=np.array([[4, 4], [100, 4]]),
        right=np.array([[4, -4], [100, -4]]),
        lane=np.array([[4, 0], [100, 0]])
    )
    road4 = Road(
        left=np.array([[-4, -100], [-4, -4]]),
        right=np.array([[4, -100], [4, -4]]),
        lane=np.array([[0, -100], [0, -4]])
    )
    l_road.extend([road1, road2, road3, road4])

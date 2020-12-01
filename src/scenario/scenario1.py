import numpy as np
from objects import (StaticObject, Road, PedestrianCross)


def scenario(l_staticObject, l_cross, l_road):
    """
    Coordinates of objects in a scenario. Include:

        l_staticObject: list of static objects

        l_cross:    list of pedestrian crosses

        l_road: list of road layouts

    """
    # # static object
    # obs1 = StaticObject(
    #     idx=1,
    #     poly=np.array([[-10, -20], [-1, -20], [-1, -4],
    #                    [-10, -4]]))
    # l_staticObject.append(obs1)

    # obs2 = StaticObject(
    #     idx=2,
    #     poly=np.array([[10, -20], [60, -20], [60, -7],
    #                    [10, -7]]))
    # l_staticObject.append(obs2)

    # obs5 = StaticObject(
    #     idx=5,
    #     poly=np.array([[-60, 7], [60, 7],
    #                    [60, 20], [-60, 20]]))
    # l_staticObject.append(obs5)

    # # pedestrian cross
    # cross1 = PedestrianCross(
    #     left=np.array([[0, -7], [0, 7]]),
    #     right=np.array([[4, -7], [4, 7]]),
    #     density=0.5
    # )
    # l_cross.append(cross1)

    # # road layout
    # road = Road(
    #     left=np.array([[-100, 4], [100, 4]]),
    #     right=np.array([[-100, -4], [100, -4]]),
    #     lane=np.array([[-100, 0], [100, 0]])
    # )
    # l_road.append(road)

    # static object
    obs1 = StaticObject(
        idx=1,
        poly=np.array([[-40, -20], [-2, -20], [-2, -5],
                       [-40, -5]]))
    l_staticObject.append(obs1)

    obs2 = StaticObject(
        idx=2,
        poly=np.array([[5, -20], [60, -20], [60, -5],
                       [5, -5]]))
    l_staticObject.append(obs2)

    obs3 = StaticObject(
        idx=3,
        poly=np.array([[-40, 20], [-2, 20], [-2, 8],
                       [-40, 8]]))
    l_staticObject.append(obs3)

    obs4 = StaticObject(
        idx=4,
        poly=np.array([[5, 20], [60, 20], [60, 5],
                       [5, 5]]))
    l_staticObject.append(obs4)

    # pedestrian cross
    cross1 = PedestrianCross(
        left=np.array([[0, -10], [0, 8]]),
        right=np.array([[4, -10], [4, 8]]),
        density=0.5
    )
    l_cross.append(cross1)

    # road layout
    road = Road(
        left=np.array([[-100, 2], [100, 2]]),
        right=np.array([[-100, -4], [100, -4]]),
        lane=np.array([[-100, -1], [100, -1]])
    )
    l_road.append(road)

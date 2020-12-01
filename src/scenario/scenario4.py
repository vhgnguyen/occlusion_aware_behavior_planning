import numpy as np
from objects import (StaticObject, Road, PedestrianCross)


def scenario(l_staticObject, l_cross, l_road):
    """
    Coordinates of objects in a scenario. Include:

        l_staticObject: list of static objects

        l_cross:    list of pedestrian crosses

        l_road: list of road layouts

    """

     # road layout
    road = Road(
        left=np.array([[-100, 2], [100, 2]]),
        right=np.array([[-100, -4], [100, -4]]),
        lane=np.array([[-100, -1], [100, -1]])
    )
    l_road.append(road)

    sx = [-20, 180]
    sy = [-10, -4]
    sz = 5
    step = 7

    idx = 1
    for i in range(sx[0], sx[1], step):
        if i > 10 and i < 40:
            continue
        idx += 1
        obs = StaticObject(
            idx=idx,
            poly=np.array([[i, sy[0]], [i+sz, sy[0]],
                          [i+sz, sy[1]], [i, sy[1]]]))
        l_staticObject.append(obs)

    obs2 = StaticObject(
        idx=21,
        poly=np.array([[-60, -20], [-25, -20], [-25, -5],
                       [-60, -5]]))
    l_staticObject.append(obs2)

    obs3 = StaticObject(
        idx=31,
        poly=np.array([[-40, 20], [-2, 20], [-2, 8],
                       [-40, 8]]))
    l_staticObject.append(obs3)

    obs4 = StaticObject(
        idx=41,
        poly=np.array([[5, 20], [60, 20], [60, 5],
                       [5, 5]]))
    l_staticObject.append(obs4)



    # # static object
    # obs11 = StaticObject(
    #     idx=11,
    #     poly=np.array([[-40, -20], [-15, -20], [-14, -10],
    #                    [-20, -6], [-40, -6]]))
    # l_staticObject.append(obs11)

    # obs12 = StaticObject(
    #     idx=12,
    #     poly=np.array([[-80, -20], [-43, -20], [-43, -8],
    #                    [-50, -6], [-70, -6]]))
    # l_staticObject.append(obs12)

    # obs13 = StaticObject(
    #     idx=13,
    #     poly=np.array([[-25, -70], [-16, -23], [-40, -23],
    #                    [-49, -70]]))
    # l_staticObject.append(obs13)

    # obs21 = StaticObject(
    #     idx=21,
    #     poly=np.array([[5, -20], [30, -20], [30, -5], [8, -5]]))
    # l_staticObject.append(obs21)

    # obs22 = StaticObject(
    #     idx=22,
    #     poly=np.array([[4, -23], [10, -23], [8, -50],
    #                    [-2, -50]]))
    # l_staticObject.append(obs22)

    # obs23 = StaticObject(
    #     idx=23,
    #     poly=np.array([[33, -20], [50, -20], [50, -5], [33, -5]]))
    # l_staticObject.append(obs23)

    # obs31 = StaticObject(
    #     idx=31,
    #     poly=np.array([[-80, 5], [-70, 5], [-63, 8],
    #                    [-63, 20], [-80, 20]]))
    # l_staticObject.append(obs31)

    # obs32 = StaticObject(
    #     idx=32,
    #     poly=np.array([[-60, 5], [-50, 5], [-43, 8],
    #                    [-43, 20], [-60, 20]]))
    # l_staticObject.append(obs32)

    # obs33 = StaticObject(
    #     idx=33,
    #     poly=np.array([[-40, 5], [-30, 5], [-23, 8],
    #                    [-23, 20], [-40, 20]]))
    # l_staticObject.append(obs33)

    # obs34 = StaticObject(
    #     idx=34,
    #     poly=np.array([[-20, 5], [-15, 5], [-10, 8],
    #                    [-6, 20], [-20, 20]]))
    # l_staticObject.append(obs34)

    # obs35 = StaticObject(
    #     idx=35,
    #     poly=np.array([[-20, 23], [-6, 23], [-6, 35],
    #                    [-8, 35], [-20, 35]]))
    # l_staticObject.append(obs35)

    # obs41 = StaticObject(
    #     idx=41,
    #     poly=np.array([[10, 7], [20, 7], [20, 20],
    #                    [11, 20]]))
    # l_staticObject.append(obs41)

    # obs42 = StaticObject(
    #     idx=42,
    #     poly=np.array([[12, 23], [20, 23], [20, 40],
    #                    [14, 40]]))
    # l_staticObject.append(obs42)

    # obs43 = StaticObject(
    #     idx=43,
    #     poly=np.array([[23, 7], [40, 7], [40, 30],
    #                    [23, 30]]))
    # l_staticObject.append(obs43)

    # # pedestrian cross
    # cross1 = PedestrianCross(
    #     left=np.array([[-10, -6], [-6, 6]]),
    #     right=np.array([[-8, -6], [-4, 6]]),
    #     density=0.8
    # )
    # cross2 = PedestrianCross(
    #     left=np.array([[2, -6], [6, 6]]),
    #     right=np.array([[4, -6], [8, 6]]),
    #     density=0.8
    # )
    # l_cross.append(cross1)
    # l_cross.append(cross2)

    # # road layout
    # road1 = Road(
    #     left=np.array([[-100, 4], [-4, 4]]),
    #     right=np.array([[-100, -4], [-6, -4]]),
    #     lane=np.array([[-100, 0], [-4, 0]])
    # )
    # road2 = Road(
    #     left=np.array([[-4, 4], [16, 100]]),
    #     right=np.array([[4, 4], [24, 100]]),
    #     lane=np.array([[0, 4], [20, 100]])
    # )
    # road3 = Road(
    #     left=np.array([[4, 4], [100, 4]]),
    #     right=np.array([[2, -4], [100, -4]]),
    #     lane=np.array([[4, 0], [100, 0]])
    # )
    # road4 = Road(
    #     left=np.array([[-26, -100], [-6, -4]]),
    #     right=np.array([[-18, -100], [2, -4]]),
    #     lane=np.array([[-22, -100], [-2, -4]])
    # )
    # l_road.extend([road1, road2, road3, road4])

import scenario1, scenario2, scenario3, scenario4


def setScenario(nr):
    """
    Interface to generate pre-defined scenarios
    """
    l_staticObject, l_cross, l_road = [], [], []
    if nr == 1:
        scenario1.scenario(l_staticObject, l_cross, l_road)
    if nr == 2:
        scenario2.scenario(l_staticObject, l_cross, l_road)
    if nr == 3:
        scenario3.scenario(l_staticObject, l_cross, l_road)
    if nr == 4:
        scenario4.scenario(l_staticObject, l_cross, l_road)

    return l_staticObject, l_cross, l_road

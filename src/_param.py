# ------------------------------
# Discreted time interval
# ------------------------------
_dT = 0.2
_PREDICT_TIME = 2
_PREDICT_STEP = 0.2
_SIMULATION_TIME = 20.0

# ------------------------------
# Environment
# ------------------------------
_SCAN_RADIUS = 35.
_FOV_ANGLE = 3.14159 / 2 - 0.1
_FOV_RAYS = 100
_CAR_LENGTH = 3.5
_CAR_WIDTH = 2

# hypothesen
_PEDESTRIAN_VX = 2.0
_VEHICLE_VX = 8

# ------------------------------
# Vehicle
# ------------------------------

# acceleration
_A_MAX = 2
_A_MIN = -2
_A_MAX_BRAKE = -6.0

# velocity
_ALPHA_V_LONG = 0.2
_ALPHA_V_LAT = 0.1

# ------------------------------
# Risk model
# ------------------------------

# static obstacle risk model
_V_MAX_OBJECT = 4
_A_MAX_BRAKE = -5

# collision orthogonal threshold
_COLLISION_ORTHO_THRES = 0.3  # 15 degree

# collision severity
_SEVERITY_MIN_WEIGHT_CONST = 10

_SEVERITY_QUAD_WEIGHT = 0.10

_SEVERITY_SIG_MAX = 10.
_SEVERITY_SIG_AVG_VX = 6.
_SEVERITY_SIG_B = 1

_SEVERITY_GOM_MAX = 10.


# collision event rate
_COLLISION_RATE_MAX = 5.
_COLLISION_RATE_BETA = 2.
# escape rate
_ESCAPE_RATE = 1.

# utility weight
_C_CRUISE = 0.001
_C_V_CRUISE = 8
_C_COMFORT = 0.001
_C_JERK = 0.01

# unseen event rate
_UNSEENEVENT_RATE_MAX = 1
_UNSEENEVENT_RATE_BETA = 0.1

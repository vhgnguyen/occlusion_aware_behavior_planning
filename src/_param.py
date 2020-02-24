# ------------------------------
# Discreted time interval
# ------------------------------
_dT = 0.2
_PREDICT_TIME = 3
_SIMULATION_TIME = 20.

# ------------------------------
# Environment
# ------------------------------
_SCAN_RADIUS = 25.

# ------------------------------
# Vehicle
# ------------------------------

# shape
_CAR_LENGTH = 4
_CAR_WIDTH = 2

# acceleration
_A_MAX = 2
_A_MIN = -2
_A_MAX_BRAKE = -5.0

# velocity
_ALPHA_V_LONG = 0.1
_ALPHA_V_LAT = 0.0

# ------------------------------
# Risk model
# ------------------------------

# static obstacle risk model
_V_MAX_OBJECT = 4
_A_MAX_BRAKE = -5

# collision orthogonal threshold
_COLLISION_ORTHO_THRES = 0.3  # 15 degree

# collision severity
_SEVERITY_QUAD_WEIGHT = 0.10
_SEVERITY_SIG_MAX = 25.
_SEVERITY_SIG_AVG_VX = 6.
_SEVERITY_MIN_WEIGHT_CONST = 10.

# collision event rate
_COLLISION_RATE_MAX = 10.
_COLLISION_RATE_BETA = 2.
# escape rate
_ESCAPE_RATE = 3.

# utility weight
_C_CRUISE = 0.001
_C_V_CRUISE = 8
_C_COMFORT = 0.001
_C_JERK = 0.0005

# unseen event rate
_UNSEENEVENT_RATE_MAX = 1
_UNSEENEVENT_RATE_BETA = 2

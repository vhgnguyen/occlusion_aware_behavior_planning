# ------------------------------
# Discreted time interval
# ------------------------------
_dT = 0.2
_PREDICT_TIME = 3
_PREDICT_STEP = 0.4
_SIMULATION_TIME = 20.0

# ------------------------------
# Environment
# ------------------------------
_SCAN_RADIUS = 50.
_FOV_ANGLE = 3.14159 - 0.001
_FOV_RAYS = 200

_CAR_LENGTH = 3.5
_CAR_WIDTH = 2

# hypothesen
_ENABLE_HYPOTHESIS = True
_PEDESTRIAN_VX = 2.0
_VEHICLE_VX = 8

# ------------------------------
# Vehicle
# ------------------------------

# acceleration
_A_MAX = 3
_A_MIN = -3
_A_MAX_BRAKE = -6.0

# velocity
_ALPHA_V_LONG = 0.2
_ALPHA_V_LAT = 0.05

# ------------------------------
# Risk model
# ------------------------------

# static obstacle risk model
_V_MAX_OBJECT = 4

# collision orthogonal threshold
_COLLISION_ORTHO_THRES = 0.3  # 15 degree

# collision severity
_SEVERITY_MIN_WEIGHT_CONST = 1.

_SEVERITY_QUAD_WEIGHT = 0.10

_SEVERITY_SIG_MAX = 5.
_SEVERITY_SIG_AVG_VX = 10.
_SEVERITY_SIG_B = 1.

_SEVERITY_GOM_MAX = 5.

# severity vehicle hypothesis
_SEVERITY_HYPOVEH_AVG_VX = 10.
_SEVERITY_HYPOVEH_MIN_WEIGHT = 2.
_SEVERITY_HYPOVEH_SIG_MAX = 3.
_SEVERITY_HYPOVEH_SIG_B = 1.

# severity pedestrian hypothesis
_SEVERITY_HYPOPEDES_AVG_VX = 5.
_SEVERITY_HYPOPEDES_MIN_WEIGHT = 1.
# gompertz
_SEVERITY_HYPOPEDES_GOM_MAX = 3.
_SEVERITY_HYPOPEDES_GOM_BETA = 4.
# sigmoid
_SEVERITY_HYPOPEDES_SIG_MAX = 3.
_SEVERITY_HYPOPEDES_SIG_BETA = 1.

# collision event rate
_COLLISION_RATE_MAX = 10.
_COLLISION_HYPOPEDES_RATE_MAX = 2.
_COLLISION_HYPOVEH_RATE_MAX = 3.
_COLLISION_RATE_EXP_BETA = 3.
_COLLISION_RATE_EXP_BETA_PEDES = 3.
_COLLISION_RATE_SIG_BETA = 10.

# escape rate
_ESCAPE_RATE = 1.

# utility weight
_C_CRUISE = 0.001
_C_V_CRUISE = 8
_C_COMFORT = 0.001
_C_JERK = 0.0005

# unseen event rate
_UNSEENEVENT_RATE_MAX = 1
_UNSEENEVENT_RATE_BETA = 0.1

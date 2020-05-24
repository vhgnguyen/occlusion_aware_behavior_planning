# ------------------------------
# Model
# ------------------------------
_ENABLE_HYPOTHESIS = True

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
_FOV_RAYS = 100

# ------------------------------
# Ego Vehicle
# ------------------------------
# car size
_CAR_LENGTH = 3.5
_CAR_WIDTH = 2
# acceleration
_A_MAX = 3
_A_MIN = -3
_A_MAX_BRAKE = -6.0
# jerk
_J_MAX = 2
# velocity uncertainty
_ALPHA_V_LONG = 0.2
_ALPHA_V_LAT = 0.05

# ------------------------------
# Collision model
# ------------------------------
# collision orthogonal threshold
_COLLISION_ORTHO_THRES = 0.3  # 15 degree

# --- Collision severity -------
_SEVERITY_MIN_WEIGHT_CONST = 5.
_COLLISION_SEVERITY_MODEL = 'sigmoid'
# quadratic model
_SEVERITY_QUAD_WEIGHT = 0.10
# sigmoid model
_SEVERITY_SIG_MAX = 5.
_SEVERITY_SIG_AVG_VX = 10.
_SEVERITY_SIG_B = 1.
# gompertz model
_SEVERITY_GOM_MAX = 5.
_SEVERITY_GOM_AVG_VX = 10.
_SEVERITY_GOM_BETA = 4.

# --- Collision event rate -----
# collision emergency brake minimum rate
_COLLISION_RATE_BRAKE_MIN = 5.
_COLLISION_RATE_MAX = 10.
_COLLISION_EVENT_RATE_MODEL = 'exponential'
# exponential model
_COLLISION_RATE_EXP_BETA = 3.
_COLLISION_RATE_EXP_BETA_PEDES = 3.
# sigmoid model
_COLLISION_RATE_SIG_BETA = 10.

# escape rate
_ESCAPE_RATE = 1.

# --- Utility weight -----------
_C_CRUISE = 0.001
_C_V_CRUISE = 8
_C_COMFORT = 0.001
_C_JERK = 0.005

# ------------------------------
# Hypothesis pedestrian model
# ------------------------------
# --- Maximum event rate -------
_COLLISION_HYPOPEDES_RATE_MAX = 2.
# --- Hypothesis covariance
_HYPOPEDES_COV_LON = 1
_HYPOPEDES_COV_LAT = 1
# --- Hypothesis velocity ------
_HYPOPEDES_VX = 2.0
# _HYPOPEDES_OFFSET_VX = 2.0
# --- Hypothesis appear rate ---
_PEDES_APPEAR_RATE_CROSS = 1
_PEDES_APPEAR_RATE_STREET = 0.3
_PEDES_APPEAR_RATE_OTHER = 0.1
_PEDES_OTHER_MIN_THRESHOLD = 5
# --- Event rate model ---------
_EVENT_RATE_HYPOPEDES_MODEL = 'sigmoid'
# --- Severity model -----------
_SEVERITY_HYPOPEDES_MODEL = 'gompertz'
_SEVERITY_HYPOPEDES_AVG_VX = 5.
_SEVERITY_HYPOPEDES_MIN_WEIGHT = 2.
# sigmoid severity
_SEVERITY_HYPOPEDES_SIG_MAX = 5.
_SEVERITY_HYPOPEDES_SIG_BETA = 1.
# gompertz severity
_SEVERITY_HYPOPEDES_GOM_MAX = 5.
_SEVERITY_HYPOPEDES_GOM_BETA = 4.

# ------------------------------
# Hypothesis vehicle model
# ------------------------------
# --- Maximum event rate -------
_COLLISION_HYPOVEH_RATE_MAX = 3.
# --- Hypothesis covariance
_HYPOVEH_COV_LON = 1
_HYPOVEH_COV_LAT = 0.5
# --- Hypothesis velocity ------
_HYPOVEH_VX = 8
# --- Hypothesis appear rate ---
_APPEAR_RATE_VEH = 1
# --- Event rate model ---------
_EVENT_RATE_HYPOVEH_MODEL = 'exponential'
# --- Severity model -----------
_SEVERITY_HYPOVEH_MODEL = 'sigmoid'
_SEVERITY_HYPOVEH_AVG_VX = 10.
_SEVERITY_HYPOVEH_MIN_WEIGHT = 2.
# sigmoid severity
_SEVERITY_HYPOVEH_SIG_MAX = 5.
_SEVERITY_HYPOVEH_SIG_B = 1


# unseen event rate
_UNSEENEVENT_RATE_MAX = 1
_UNSEENEVENT_RATE_BETA = 0.1
_V_MAX_OBJECT = 4
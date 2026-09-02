import os

# Single source of truth for every constant the delta robot subsystem
# uses - geometry, joint limits, workspace/operating envelope, camera
# placeholder numbers, actuator/PID parameters, and where results get
# written. Everything else (kinematics.py, integration.py,
# test_data_generator.py) just does `import config as cfg` and reads
# from here so a number only ever has to change in one place.

# --- Delta geometry (mm) - Clavel-style parallel delta -----------------
# Matches the physical layout described in the paper: 150mm base,
# 50mm end-effector platform, 120mm actuated upper arm, 230mm passive
# lower-arm parallelogram.
BASE_SIDE = 150.0
PLATFORM_SIDE = 50.0
UPPER_ARM = 120.0
LOWER_ARM = 230.0

# --- Joint limits --------------------------------------------------
# MG996R-class hobby servos are geared for roughly +/-90 deg of usable
# travel before horn/linkage geometry starts binding, so that's the
# window we enforce in inverse(). Keep this a good bit inside the servo's
# absolute mechanical stop, not right up against it.
THETA_MIN = -90.0
THETA_MAX = 90.0

# Minimum distance allowed between any two wrist points before we call
# it a self-collision risk (upper arms clipping each other near full
# extension). Not a measured number - a conservative stand-in until
# there's a real arm to check clearance against.
MIN_WRIST_CLEARANCE = 40.0

# --- Reachable / operating envelope ---------------------------------
# OPERATING_RADIUS is the conservative planning envelope (weeds get
# generated inside this), which is intentionally smaller than the
# ~253mm max reach the workspace sampling in kinematics.py actually
# finds - we don't want to be planning right up against the edge of
# what the arm can physically do.
OPERATING_RADIUS = 200.0
Z_MIN = -220.0
Z_MAX = -140.0

MIN_WEED_SEPARATION = 15.0  # mm, so the cutter has room to work without re-centering

# --- Camera placeholder (no hardware mounted/calibrated yet) --------
CAM_X_RANGE = (-200.0, 200.0)
CAM_Y_RANGE = (-200.0, 200.0)
CAM_Z_NOMINAL = -170.0
CAM_Z_NOISE = 10.0

# --- Base motion / cycle-time budget --------------------------------
JOINT_SPEED_DEG_S = 200.0  # derated from no-load servo spec, arm is under load
SETTLE_TIME_S = 0.05
CUT_DWELL_S = 0.30

# --- Actuator dynamics (MG996R-class servo, simulated) ---------------
# J is the arm treated as a slender rod (J = 1/3 * m * L^2) about the
# joint, then bumped up with a multiplier to account for the gearbox and
# lower-arm loading we're not modeling explicitly. The multiplier is an
# engineering guess, not something derived - flagged as such in the
# paper.
_ARM_MASS_KG = 0.1
_ARM_INERTIA_MULTIPLIER = 3.0
ACTUATOR_J = _ARM_INERTIA_MULTIPLIER * (1.0 / 3.0) * _ARM_MASS_KG * (UPPER_ARM / 1000.0) ** 2

# Viscous damping estimated from a representative servo datasheet:
# b = stall_torque / no_load_speed
_STALL_TORQUE_NM = 0.92       # ~9.4 kg*cm at 6V, converted
_NO_LOAD_SPEED_RAD_S = 6.16   # ~0.17 s/60 deg at 6V, converted
ACTUATOR_B = _STALL_TORQUE_NM / _NO_LOAD_SPEED_RAD_S
TORQUE_LIMIT_NM = _STALL_TORQUE_NM

PID_KP = 0.08
PID_KI = 0.02
PID_KD = 0.008
PID_TARGET_DEG = 45.0
PID_DT_S = 0.001
PID_SIM_DURATION_S = 0.5

# --- Output location --------------------------------------------------
RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
os.makedirs(RESULTS_DIR, exist_ok=True)


def results_path(filename):
    return os.path.join(RESULTS_DIR, filename)

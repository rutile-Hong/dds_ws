from dataclasses import dataclass


# ---------------------------------------------------
# Command sent from controller to the ROS node
# ---------------------------------------------------

@dataclass
class ControlCommand:
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    thrust: float = 0.0

    joy_roll: float = 0.0
    joy_pitch: float = 0.0
    joy_yaw: float = 0.0
    joy_throttle: float = -1.0


# ---------------------------------------------------
# Desired controller states
# ---------------------------------------------------

@dataclass
class ReferenceState:
    y: float = 0.0
    z: float = 0.0

    vy: float = 0.0
    vz: float = 0.0


# ---------------------------------------------------
# Controller outputs (before conversion to Joy)
# ---------------------------------------------------

@dataclass
class ControllerOutput:
    roll_deg: float = 0.0
    thrust: float = 0.0

    vy_desired: float = 0.0
    vz_desired: float = 0.0


# ---------------------------------------------------
# Actual vehicle state used for logging
# ---------------------------------------------------

@dataclass
class ActualLog:

    count: int
    timestamp: int

    qx: float
    qy: float
    qz: float
    qw: float

    p: float
    q: float
    r: float

    ax: float
    ay: float
    az: float

    x: float
    y: float
    z: float

    vx: float
    vy: float
    vz: float

    roll: float
    pitch: float
    yaw: float

    y_actual: float
    vy_actual: float


# ---------------------------------------------------
# Control information for logging
# ---------------------------------------------------

@dataclass
class ControlLog:

    count: int
    timestamp: int

    y: float
    y_target: float

    z: float
    z_target: float

    vy: float
    vy_target: float

    vz: float
    vz_target: float

    roll: float
    roll_target: float

    pitch: float
    yaw: float

    thrust: float

    joy_roll: float
    joy_pitch: float
    joy_throttle: float
    joy_yaw: float

    dt: float
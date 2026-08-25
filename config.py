from dataclasses import dataclass


# ---------------------------------------------------
# Control loop
# ---------------------------------------------------

@dataclass(frozen=True)
class ControlConfig:
    frequency: float = 20.0
    hold_delay: float = 5.0          # seconds before capturing hold position


# ---------------------------------------------------
# Vehicle
# ---------------------------------------------------

@dataclass(frozen=True)
class VehicleConfig:
    hover_thrust: float = 0.4

    thrust_min: float = 0.0
    thrust_max: float = 1.0

    max_roll_deg: float = 45.0


# ---------------------------------------------------
# Manual motion
# ---------------------------------------------------

@dataclass(frozen=True)
class ManualConfig:
    y_step: float = 0.05
    z_step: float = 0.05


# ---------------------------------------------------
# Logging
# ---------------------------------------------------

@dataclass(frozen=True)
class LoggingConfig:
    directory: str = "logs"

    actual_filename: str = "data_actual.csv"
    control_filename: str = "data_control.csv"

    flush_every: int = 20


# ---------------------------------------------------
# QoS
# ---------------------------------------------------

@dataclass(frozen=True)
class QoSConfig:
    depth: int = 100


# ---------------------------------------------------
# PID
# ---------------------------------------------------

@dataclass(frozen=True)
class PIDGains:

    # Position loop
    y_kp: float = 0.8  #0.9 15V,
    y_ki: float = 0.0   #0.05
    y_kd: float = 0.0

    z_kp: float = 3.0
    z_ki: float = 0.0
    z_kd: float = 0.0

    # Velocity loop
    vy_kp: float = 3.0  #4.0 15V
    vy_ki: float = 1.8  #1.5 15V, 2.0: decrease
    vy_kd: float = 0.2   #0.1

    vz_kp: float = 0.35
    vz_ki: float = 0.35
    vz_kd: float = 0.02
    
    # Roll angle -> roll rate
    roll_angle_kp = 4.0
    
    # Roll rate PID
    rate_roll_kp = 0.15
    rate_roll_ki = 0.05
    rate_roll_kd = 0.002

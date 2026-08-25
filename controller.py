#!/usr/bin/env python3

"""
controller.py

Cascade PID controller for the bicopter.

Architecture


Y-axis / Roll Control
---------------------

Reference Y Position
        |
        V
 Position PID
        |
        V
 Desired Y Velocity
        |
        V
 Velocity PID
        |
        V
 Desired Lateral Acceleration
        |
        V
 Desired Roll Angle
        |
        V
 Roll Angle P Controller
        |
        V
 Desired Roll Rate
        |
        V
 Roll Rate PID
        |
        V
 Roll Control Command


Z-axis Control
--------------

Reference Z Position
        |
        V
 Position PID
        |
        V
 Desired Z Velocity
        |
        V
 Velocity Z PID
        |
        V
 Thrust
"""

import math

from tf_transformations import euler_from_quaternion

from PID_Controller import PIDController

from config import (
    PIDGains,
    VehicleConfig,
)

from messages import (
    ReferenceState,
    ControllerOutput,
)


class CascadePIDController:

    def __init__(self):

        gains = PIDGains()
        vehicle = VehicleConfig()

        #################################################
        # Vehicle parameters
        #################################################

        self.hover_thrust = vehicle.hover_thrust

        self.max_roll_deg = vehicle.max_roll_deg

        self.thrust_min = vehicle.thrust_min
        self.thrust_max = vehicle.thrust_max

        #################################################
        # Roll controller parameters
        #################################################

        # Roll angle error [rad]
        # ->
        # desired roll rate [rad/s]
        self.roll_angle_kp = gains.roll_angle_kp

        # Maximum roll-rate command
        self.max_roll_rate = math.radians(
            vehicle.max_roll_rate_deg
        )

        #################################################
        # Position PIDs
        #################################################

        self.pid_y = PIDController(
            Kp=gains.y_kp,
            Ki=gains.y_ki,
            Kd=gains.y_kd,
            output_limits=(-20.0, 20.0),
        )

        self.pid_z = PIDController(
            Kp=gains.z_kp,
            Ki=gains.z_ki,
            Kd=gains.z_kd,
            output_limits=(-0.3, 0.3),
        )

        #################################################
        # Velocity PIDs
        #################################################

        # Y velocity PID output:
        #
        # desired lateral acceleration [m/s^2]
        self.pid_vy = PIDController(
            Kp=gains.vy_kp,
            Ki=gains.vy_ki,
            Kd=gains.vy_kd,
            output_limits=(-9.0, 9.0),
        )

        # Z velocity PID output:
        #
        # thrust correction
        self.pid_vz = PIDController(
            Kp=gains.vz_kp,
            Ki=gains.vz_ki,
            Kd=gains.vz_kd,
            output_limits=(-0.2, 0.2),
        )

        #################################################
        # Roll Rate PID
        #################################################

        # Input:
        # desired roll rate - actual roll rate
        #
        # Output:
        # normalized roll-control command
        self.pid_roll_rate = PIDController(
            Kp=gains.rate_roll_kp,
            Ki=gains.rate_roll_ki,
            Kd=gains.rate_roll_kd,
            output_limits=(-1.0, 1.0),
        )

    ###########################################################
    # Public API
    ###########################################################

    def reset(self):

        #################################################
        # Position controllers
        #################################################

        self.pid_y.reset()
        self.pid_z.reset()

        #################################################
        # Velocity controllers
        #################################################

        self.pid_vy.reset()
        self.pid_vz.reset()

        #################################################
        # Rate controller
        #################################################

        self.pid_roll_rate.reset()

    ###########################################################
    # Utility Functions
    ###########################################################

    @staticmethod
    def quaternion_to_euler(state):

        return euler_from_quaternion(
            [
                state.qx,
                state.qy,
                state.qz,
                state.qw,
            ]
        )

    @staticmethod
    def yaw_compensated_position(
        state,
        yaw,
    ):

        return (
            -state.x * math.sin(yaw)
            + state.y * math.cos(yaw)
        )

    @staticmethod
    def yaw_compensated_velocity(
        state,
        yaw,
    ):

        return (
            -state.vx * math.sin(yaw)
            + state.vy * math.cos(yaw)
        )

    @staticmethod
    def clamp(
        value,
        minimum,
        maximum,
    ):

        if value < minimum:
            return minimum

        if value > maximum:
            return maximum

        return value

    ###########################################################
    # Position Loop
    ###########################################################

    def position_controller(
        self,
        y,
        z,
        reference: ReferenceState,
        dt,
    ):

        #################################################
        # Set references
        #################################################

        self.pid_y.setpoint = reference.y
        self.pid_z.setpoint = reference.z

        #################################################
        # Y position -> desired Y velocity
        #################################################

        vy_desired = self.pid_y.update(
            y,
            dt,
        )

        #################################################
        # Z position -> desired Z velocity
        #################################################

        vz_desired = self.pid_z.update(
            z,
            dt,
        )

        return (
            vy_desired,
            vz_desired,
        )

    ###########################################################
    # Velocity Loop
    ###########################################################

    def velocity_controller(
        self,
        vy,
        vz,
        vy_desired,
        vz_desired,
        dt,
    ):

        #################################################
        # Set velocity references
        #################################################

        self.pid_vy.setpoint = vy_desired
        self.pid_vz.setpoint = vz_desired

        #################################################
        # Y Velocity PID
        #
        # Output:
        #
        # desired lateral acceleration [m/s^2]
        #################################################

        ay_desired = self.pid_vy.update(
            vy,
            dt,
        )

        #################################################
        # Acceleration -> Roll
        #################################################

        g = 9.80665

        # Approximate lateral dynamics:
        #
        # ay = g * tan(phi)
        #
        # therefore:
        #
        # phi = atan(ay / g)

        roll_target = -math.atan2(
            ay_desired,
            g,
        )

        #################################################
        # Roll-angle limit
        #################################################

        max_roll_rad = math.radians(
            self.max_roll_deg
        )

        roll_target = self.clamp(
            roll_target,
            -max_roll_rad,
            max_roll_rad,
        )

        #################################################
        # Z Velocity PID -> Thrust
        #################################################

        thrust_correction = self.pid_vz.update(
            vz,
            dt,
        )

        thrust = (
            self.hover_thrust
            + thrust_correction
        )

        #################################################
        # Thrust saturation
        #################################################

        thrust = self.clamp(
            thrust,
            self.thrust_min,
            self.thrust_max,
        )

        return (
            roll_target,
            thrust,
            ay_desired,
        )

    ###########################################################
    # Roll Angle Loop
    ###########################################################

    def roll_angle_controller(
        self,
        roll_actual,
        roll_target,
    ):

        #################################################
        # Roll angle error
        #################################################

        roll_error = (
            roll_target
            - roll_actual
        )

        #################################################
        # Angle P controller
        #
        # angle error [rad]
        #
        # ->
        #
        # desired angular rate [rad/s]
        #################################################

        roll_rate_target = (
            self.roll_angle_kp
            * roll_error
        )

        #################################################
        # Roll-rate target saturation
        #################################################

        roll_rate_target = self.clamp(
            roll_rate_target,
            -self.max_roll_rate,
            self.max_roll_rate,
        )

        return roll_rate_target

    ###########################################################
    # Roll Rate Loop
    ###########################################################

    def roll_rate_controller(
        self,
        roll_rate_actual,
        roll_rate_target,
        dt,
    ):

        #################################################
        # Set rate target
        #################################################

        self.pid_roll_rate.setpoint = (
            roll_rate_target
        )

        #################################################
        # Roll-rate PID
        #################################################

        roll_control = self.pid_roll_rate.update(
            roll_rate_actual,
            dt,
        )

        return roll_control

    ###########################################################
    # Main Controller
    ###########################################################

    def update(
        self,
        state,
        reference: ReferenceState,
        dt,
    ):

        #################################################
        # Invalid timestep
        #################################################

        if dt <= 0.0:

            return ControllerOutput()

        #################################################
        # Convert attitude
        #################################################

        roll, pitch, yaw = (
            self.quaternion_to_euler(
                state
            )
        )

        #################################################
        # Yaw-compensated Y motion
        #################################################

        y_actual = (
            self.yaw_compensated_position(
                state,
                yaw,
            )
        )

        vy_actual = (
            self.yaw_compensated_velocity(
                state,
                yaw,
            )
        )

        #################################################
        # Position Controller
        #################################################

        (
            vy_desired,
            vz_desired,
        ) = self.position_controller(
            y_actual,
            state.z,
            reference,
            dt,
        )

        #################################################
        # Velocity Controller
        #
        # Output:
        #
        # desired roll
        # thrust
        # desired lateral acceleration
        #################################################

        (
            roll_target,
            thrust,
            ay_desired,
        ) = self.velocity_controller(
            vy_actual,
            state.vz,
            vy_desired,
            vz_desired,
            dt,
        )

        #################################################
        # Roll Angle Controller
        #
        # desired roll
        #
        # ->
        #
        # desired roll rate
        #################################################

        roll_rate_target = (
            self.roll_angle_controller(
                roll_actual=roll,
                roll_target=roll_target,
            )
        )

        #################################################
        # Actual Roll Rate
        #################################################

        # IMPORTANT:
        #
        # Roll rate is body-X angular velocity.
        #
        # This assumes your state message contains:
        #
        # state.wx [rad/s]
        #
        # If your variable is instead state.p,
        # replace this line with:
        #
        # roll_rate_actual = state.p

        roll_rate_actual = state.wx

        #################################################
        # Roll Rate PID
        #################################################

        roll_control = (
            self.roll_rate_controller(
                roll_rate_actual,
                roll_rate_target,
                dt,
            )
        )

        #################################################
        # Convert diagnostic values to degrees
        #################################################

        roll_deg = math.degrees(
            roll_target
        )

        roll_rate_target_deg = math.degrees(
            roll_rate_target
        )

        roll_rate_actual_deg = math.degrees(
            roll_rate_actual
        )

        #################################################
        # Generate Output
        #################################################

        # IMPORTANT:
        #
        # At this point there are TWO fundamentally
        # different outputs:
        #
        # roll_target
        #     desired attitude
        #
        # roll_control
        #     output of YOUR roll-rate PID
        #
        # If you want your own rate controller to control
        # the motors, roll_control must eventually be used
        # in the motor mixer / DirectPWM command.
        #
        # Merely sending roll_deg to ArduPilot means
        # ArduPilot will still use its own inner-loop
        # attitude/rate controllers.

        return ControllerOutput(

            roll_deg=roll_deg,

            thrust=thrust,

            vy_desired=vy_desired,

            vz_desired=vz_desired,

            # Add these if ControllerOutput supports them:
            #
            # roll_control=roll_control,
            # roll_rate_target=roll_rate_target,
            # roll_rate_actual=roll_rate_actual,
            # ay_desired=ay_desired,
        )

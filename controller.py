#!/usr/bin/env python3

"""
controller.py

Cascade PID controller for the bicopter.

Architecture

Reference Position
        |
        V
 Position PID
        |
 Desired Velocity
        |
        V
 Velocity PID
        |
 Roll + Thrust
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

        self.hover_thrust = vehicle.hover_thrust

        self.max_roll_deg = vehicle.max_roll_deg

        self.thrust_min = vehicle.thrust_min
        self.thrust_max = vehicle.thrust_max

        # Internal shaped velocity target
        self.vy_target_shaped = 0.0
        self.ay_target_shaped = 0.0
        self.trajectory_accel = 0.30   # m/s^2
        self.trajectory_jerk = 1.0     # m/s^3
        self.max_lateral_accel = 9.0  # m/s^2      # m/s^3

        #################################################
        # Position PIDs
        #################################################

        self.pid_y = PIDController(
            Kp=gains.y_kp,
            Ki=gains.y_ki,
            Kd=gains.y_kd,
            output_limits=(-20, 20),
        )

        self.pid_z = PIDController(
            Kp=gains.z_kp,
            Ki=gains.z_ki,
            Kd=gains.z_kd,
            output_limits=(-0.3,0.3),
        )

        #################################################
        # Velocity PIDs
        #################################################

        self.pid_vy = PIDController(
            Kp=gains.vy_kp,
            Ki=gains.vy_ki,
            Kd=gains.vy_kd,
            output_limits=(-9.0, 9.0),
        )

        self.pid_vz = PIDController(
            Kp=gains.vz_kp,
            Ki=gains.vz_ki,
            Kd=gains.vz_kd,
            output_limits=(-0.2, 0.2),
        )

    ###########################################################
    # Public API
    ###########################################################

    def reset(self):

        self.pid_y.reset()
        self.pid_z.reset()

        self.pid_vy.reset()
        self.pid_vz.reset()

        self.vy_target_shaped = 0.0
        self.ay_target_shaped = 0.0

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
    def yaw_compensated_position(state, yaw):

        return (
            -state.x * math.sin(yaw)
            + state.y * math.cos(yaw)
        )

    @staticmethod
    def yaw_compensated_velocity(state, yaw):

        return (
            -state.vx * math.sin(yaw)
            + state.vy * math.cos(yaw)
        )

    @staticmethod
    def clamp(value, minimum, maximum):

        if value < minimum:
            return minimum

        if value > maximum:
            return maximum

        return value

    ###########################################################
    # Velocity Target Shaping
    ###########################################################

    def shape_velocity_target(
        self,
        vy_target_raw,
        dt,
    ):
        """
        Generate a smooth velocity target and acceleration
        feed-forward using acceleration and jerk limits.

        Inputs
        ------
        vy_target_raw : float
            Raw velocity target from position controller [m/s]

        dt : float
            Controller timestep [s]

        Returns
        -------
        vy_desired : float
            Shaped velocity target [m/s]

        ay_feedforward : float
            Desired lateral acceleration feed-forward [m/s^2]
        """

        #################################################
        # Safety check
        #################################################

        if dt <= 0.0:

            return (
                self.vy_target_shaped,
                self.ay_target_shaped,
            )

        #################################################
        # Velocity error of trajectory generator
        #################################################

        velocity_error = (
            vy_target_raw
            - self.vy_target_shaped
        )

        #################################################
        # Determine requested acceleration
        #
        # Use stopping distance to decide when to
        # start reducing acceleration.
        #
        # stopping distance:
        #
        # dv = a^2 / (2*j)
        #
        #################################################

        accel_abs = abs(
            self.ay_target_shaped
        )

        if self.trajectory_jerk > 0.0:

            stopping_velocity = (
                accel_abs * accel_abs
                / (
                    2.0
                    * self.trajectory_jerk
                )
            )

        else:

            stopping_velocity = 0.0

        #################################################
        # Requested acceleration
        #################################################

        if abs(velocity_error) < 1e-6:

            # Already at target velocity
            ay_requested = 0.0

        elif (
            abs(velocity_error)
            <= stopping_velocity
        ):

            # We are getting close to the requested
            # velocity.
            #
            # Start reducing acceleration so that the
            # trajectory does not strongly overshoot.
            ay_requested = 0.0

        elif velocity_error > 0.0:

            ay_requested = (
                self.trajectory_accel
            )

        else:

            ay_requested = (
                -self.trajectory_accel
            )

        #################################################
        # Jerk limit
        #
        # jerk = da / dt
        #
        # therefore:
        #
        # max_da = jerk_max * dt
        #################################################

        max_da = (
            self.trajectory_jerk
            * dt
        )

        da_requested = (
            ay_requested
            - self.ay_target_shaped
        )

        da = self.clamp(
            da_requested,
            -max_da,
            max_da,
        )

        #################################################
        # Update shaped acceleration
        #################################################

        self.ay_target_shaped += da

        #################################################
        # Acceleration safety limit
        #################################################

        self.ay_target_shaped = self.clamp(
            self.ay_target_shaped,
            -self.trajectory_accel,
            self.trajectory_accel,
        )

        #################################################
        # Integrate acceleration -> velocity
        #################################################

        old_vy_target = (
            self.vy_target_shaped
        )

        self.vy_target_shaped += (
            self.ay_target_shaped
            * dt
        )

        #################################################
        # Prevent crossing the requested velocity target
        #
        # Example:
        #
        # requested = 0.20
        # calculated = 0.205
        #
        # clamp it back to 0.20
        #################################################

        if velocity_error > 0.0:

            if (
                self.vy_target_shaped
                > vy_target_raw
            ):

                self.vy_target_shaped = (
                    vy_target_raw
                )

                self.ay_target_shaped = 0.0

        elif velocity_error < 0.0:

            if (
                self.vy_target_shaped
                < vy_target_raw
            ):

                self.vy_target_shaped = (
                    vy_target_raw
                )

                self.ay_target_shaped = 0.0

        #################################################
        # Acceleration Feed Forward
        #################################################

        ay_feedforward = (
            self.ay_target_shaped
        )

        #################################################
        # Return trajectory states
        #################################################

        return (
            self.vy_target_shaped,
            ay_feedforward,
        )

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
        self.pid_y.setpoint = reference.y
        self.pid_z.setpoint = reference.z

        vy_target_raw = self.pid_y.update(
            y,
            dt,
        )
        vz_desired = self.pid_z.update(
            z,
            dt,
        )

        vy_desired, ay_feedforward = (
            self.shape_velocity_target(
                vy_target_raw,
                dt,
            )
        )

        return (
            vy_desired,
            vz_desired,
            ay_feedforward,
        )

    ###########################################################
    # Velocity Loop
    ###########################################################
    # def velocity_controller(
    #         self,
    #         vy,
    #         vz,
    #         vy_desired,
    #         vz_desired,
    #         dt,
    #     ):
    
    #         self.pid_vy.setpoint = vy_desired
    #         self.pid_vz.setpoint = vz_desired
    
    #         #################################################
    #         # Roll
    #         #################################################
    
    #         roll_target = -self.pid_vy.update(
    #             vy,
    #             dt,
    #         )
    
    #         #################################################
    #         # Thrust
    #         #################################################
    
    #         thrust = (
    #             self.pid_vz.update(
    #                 vz,
    #                 dt,
    #             )
    #             + self.hover_thrust
    #         )
    
    #         #################################################
    #         # Saturation
    #         #################################################
    
    #         roll_target = self.clamp(
    #             roll_target,
    #             -math.pi / 4.0,
    #             math.pi / 4.0,
    #         )
    
    #         thrust = self.clamp(
    #             thrust,
    #             self.thrust_min,
    #             self.thrust_max,
    #         )
    
    #         return (
    #             roll_target,
    #             thrust,
    #         )

    def velocity_controller(
        self,
        vy,
        vz,
        vy_desired,
        vz_desired,
        ay_feedforward,
        dt,
    ):

        self.pid_vy.setpoint = vy_desired
        self.pid_vz.setpoint = vz_desired

        #################################################
        # Y velocity PID
        #
        # Output is desired lateral acceleration [m/s^2]
        #################################################

        ay_feedback = self.pid_vy.update(
            vy,
            dt,
        )

        ay_desired = (
            ay_feedforward
            + ay_feedback
        )

        #################################################
        # Convert lateral acceleration to roll angle
        #################################################

        g = 9.80665

        # Exact relationship assuming vertical thrust
        # is adjusted to maintain altitude:
        #
        # ay = g * tan(roll)
        #
        # therefore:
        #
        # roll = atan(ay / g)

        roll_target = -math.atan2(
            ay_desired,
            g,
        )

        #################################################
        # Roll saturation
        #################################################

        roll_target = self.clamp(
                roll_target,
                -math.pi / 4.0,
                math.pi / 4.0,
            )
        
        #################################################
        # Z velocity PID -> thrust
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
        )

    ###########################################################
    # Main Controller
    ###########################################################

    def update(
        self,
        state,
        reference: ReferenceState,
        dt,
    ):

        if dt <= 0.0:

            return ControllerOutput()


            #################################################
        # Convert attitude
        #################################################

        roll, pitch, yaw = self.quaternion_to_euler(
            state
        )

        #################################################
        # Yaw compensated motion
        #################################################

        y_actual = self.yaw_compensated_position(
            state,
            yaw,
        )

        # vy_actual = self.yaw_compensated_velocity(
        #     state,
        #     yaw,
        # )
        # y_actual = state.y
        vy_actual = state.vy


        #################################################
        # Position controller
        #################################################

        vy_desired, vz_desired, ay_feedforward = self.position_controller(
            y_actual,
            state.z,
            reference,
            dt,
        )


        #################################################
        # Velocity controller
        #################################################

        roll_target, thrust = self.velocity_controller(
            vy_actual,
            state.vz,
            vy_desired,
            vz_desired,
            ay_feedforward,
            dt,
        )


        #################################################
        # Convert roll to degrees
        #################################################

        roll_deg = roll_target * 180.0 / math.pi


        #################################################
        # Generate output
        #################################################

        return ControllerOutput(

            roll_deg=roll_deg,

            thrust=thrust,


            vy_desired=vy_desired,

            vz_desired=vz_desired,

        )    

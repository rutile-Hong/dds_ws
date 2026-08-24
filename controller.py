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

        vy_desired = self.pid_y.update(
            y,
            dt,
        )
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
        dt,
    ):

        self.pid_vy.setpoint = vy_desired
        self.pid_vz.setpoint = vz_desired

        #################################################
        # Y velocity PID
        #
        # Output is desired lateral acceleration [m/s^2]
        #################################################

        ay_desired = self.pid_vy.update(
            vy,
            dt,
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

        vy_desired, vz_desired = self.position_controller(
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
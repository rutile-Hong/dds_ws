#!/usr/bin/env python3

"""
node.py

ROS2 interface for bicopter PID controller.

Responsibilities:

- Receive ArduPilot DDS data
- Maintain VehicleState
- Run controller at fixed frequency
- Publish Joy commands
- Send data to logger
"""


import math
import time
import numpy as np

import rclpy

from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)


from sensor_msgs.msg import (
    Imu,
    Joy,
)

from geometry_msgs.msg import (
    PoseStamped,
    TwistStamped,
)


from vehicle_state import VehicleState

from controller import CascadePIDController

from keyboard import KeyboardController

from logger import CSVLogger


from messages import (
    ReferenceState,
    ActualLog,
    ControlLog,
    ControllerOutput
)


from config import (
    ControlConfig,
    VehicleConfig,
    ManualConfig,
    LoggingConfig,
    QoSConfig,
)



class PIDControlDDSNode(Node):


    def __init__(self):

        super().__init__(
            "bicopter_pid_controller"
        )


        ###################################################
        # Configuration
        ###################################################

        self.control_cfg = ControlConfig()

        self.vehicle_cfg = VehicleConfig()

        self.manual_cfg = ManualConfig()

        self.logging_cfg = LoggingConfig()

        self.qos_cfg = QoSConfig()



        ###################################################
        # Internal state
        ###################################################

        self.counter = 0

        self.last_time = (
            self.get_clock().now()
        )


        self.controller_enabled = False


        self.reference_initialized = False
        self.startup_time = None
        self.startup_duration = 5.0
        self.startup_thrust = 0.10


        self.reference = ReferenceState()



        ###################################################
        # Core components
        ###################################################

        self.vehicle_state = VehicleState()


        self.controller = CascadePIDController()



        ###################################################
        # QoS
        ###################################################

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=self.qos_cfg.depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        command_qos = QoSProfile(

            history=HistoryPolicy.KEEP_LAST,

            depth=10,

            reliability=ReliabilityPolicy.RELIABLE,

            durability=DurabilityPolicy.VOLATILE,

        )



        ###################################################
        # Subscribers
        ###################################################

        self.create_subscription(

            Imu,

            "/ap/imu/experimental/data",

            self.imu_callback,

            sensor_qos,

        )


        self.create_subscription(

            PoseStamped,

            "/ap/pose/filtered",

            self.pose_callback,

            sensor_qos,

        )


        self.create_subscription(

            TwistStamped,

            "/ap/twist/filtered",

            self.twist_callback,

            sensor_qos,

        )



        ###################################################
        # Publisher
        ###################################################

        self.joy_pub = self.create_publisher(

            Joy,

            "/ap/joy",

            command_qos,

        )


        self.joy_msg = Joy()

        self.joy_msg.buttons = []



        ###################################################
        # Logger
        ###################################################

        self.logger = CSVLogger(

            actual_file=
            f"{self.logging_cfg.directory}/"
            f"{self.logging_cfg.actual_filename}",


            control_file=
            f"{self.logging_cfg.directory}/"
            f"{self.logging_cfg.control_filename}",


            flush_every=
            self.logging_cfg.flush_every,

        )


        self.setup_logger_headers()



        ###################################################
        # Keyboard
        ###################################################

        self.keyboard = KeyboardController(

            self.reference,

            self.vehicle_state,

        )


        self.keyboard.start()



        ###################################################
        # Control timer
        ###################################################

        self.timer = self.create_timer(

            1.0 /
            self.control_cfg.frequency,

            self.control_loop,

        )



        self.get_logger().info(
            "Bicopter PID DDS controller started"
        )

        self.get_logger().info(
            "Press s to enable controller"
        )


        
    ###################################################
    # ROS callbacks
    ###################################################

    def imu_callback(self, msg: Imu):

        self.vehicle_state.update_imu(msg)



    def pose_callback(self, msg: PoseStamped):

        self.vehicle_state.update_pose(msg)



    def twist_callback(self, msg: TwistStamped):

        self.vehicle_state.update_twist(msg)



    ###################################################
    # Main control loop
    ###################################################

    def control_loop(self):

        ###################################################
        # Controller disabled
        ###################################################

        if not self.keyboard.enabled:

            self.startup_time = None
            self.reference_initialized = False

            self.controller.reset()

            self.last_time = self.get_clock().now()

            return



        ###################################################
        # Emergency stop
        ###################################################

        if self.keyboard.emergency_stop:

            self.send_motor_stop()

            return



        ###################################################
        # Timing
        ###################################################

        now = self.get_clock().now()


        dt = (
            now - self.last_time
        ).nanoseconds * 1e-9


        self.last_time = now



        if dt <= 0:

            return

        dt = np.clip(dt, 0.03, 0.07)



        ###################################################
        # Get consistent sensor snapshot
        ###################################################

        state = self.vehicle_state.copy()

        ###################################################
        # Start startup timer
        ###################################################

        if self.startup_time is None:

            self.startup_time = time.monotonic()

            self.get_logger().info(
                "Startup phase: low throttle for 5 seconds"
            )

        elapsed = (
            time.monotonic()
            - self.startup_time
        )

        ###################################################
        # First 5 seconds:
        # No PID, roll = 0, low throttle
        ###################################################

        if elapsed < self.startup_duration:

            joy_values = self.publish_joy(
                roll_deg=0.0,
                thrust=self.startup_thrust,
            )

            self.get_logger().info(
                f"Startup: {elapsed:.1f} / "
                f"{self.startup_duration:.1f} sec",
                throttle_duration_sec=1.0,
            )

            return


        ###################################################
        # Initialize position hold target
        ###################################################

        if not self.reference_initialized:


            #
            # Use current position as starting point
            #

            self.reference.y = state.y
            #
            # Move slightly upward
            #

            self.reference.z = (
                state.z
                +
                self.manual_cfg.z_step
            )


            self.reference.vy = 0.0

            self.reference.vz = 0.0


            self.reference_initialized = True


            self.get_logger().info(
                f"Reference initialized "
                f"y={self.reference.y:.3f}, "
                f"z={self.reference.z:.3f}"
            )



        ###################################################
        # Controller update
        ###################################################

        command = self.controller.update(

            state,

            self.reference,

            dt,

        )


        #command.roll_deg = 0.0
        #command.thrust = 0.35
        ###################################################
        # Publish command
        ###################################################

        joy_values = self.publish_joy(

            command.roll_deg,

            command.thrust,

        )



        ###################################################
        # Logging
        ###################################################

        self.create_logs(

            state,

            command,

            joy_values,

            dt,

        )


        self.counter += 1



    ###################################################
    # Joy conversion and publishing
    ###################################################

    def publish_joy(
        self,
        roll_deg,
        thrust,
    ):


        ###################################################
        # Convert roll angle to joystick scale
        ###################################################

        joy_roll = (

            roll_deg /
            self.vehicle_cfg.max_roll_deg

        )


        joy_roll = max(
            -1.0,
            min(
                1.0,
                joy_roll
            )
        )



        ###################################################
        # Convert thrust
        ###################################################

        joy_throttle = (

            2.0 *
            (
                thrust
                -
                self.vehicle_cfg.thrust_min
            )
            /
            (
                self.vehicle_cfg.thrust_max
                -
                self.vehicle_cfg.thrust_min
            )

            -
            1.0

        )


        joy_throttle = max(
            -1.0,
            min(
                1.0,
                joy_throttle
            )
        )



        ###################################################
        # Reuse Joy object
        ###################################################

        self.joy_msg.header.stamp = (

            self.get_clock()
            .now()
            .to_msg()

        )


        self.joy_msg.axes = [

            float(joy_roll),      # roll

            float("nan"),                  # pitch

            float(joy_throttle),  # throttle

            float("nan"),                  # yaw

        ]



        self.joy_pub.publish(

            self.joy_msg

        )



        return (

            joy_roll,

            0.0,

            joy_throttle,

            0.0,

        )

    
    ###################################################
    # Logger headers
    ###################################################

    def setup_logger_headers(self):


        actual_header = [

            "count",
            "timestamp",

            "qx",
            "qy",
            "qz",
            "qw",

            "p",
            "q",
            "r",

            "ax",
            "ay",
            "az",

            "x",
            "y",
            "z",

            "vx",
            "vy",
            "vz",

            "roll",
            "pitch",
            "yaw",

            "y_actual",
            "vy_actual",

        ]


        control_header = [

            "count",
            "timestamp",

            "y",
            "y_target",

            "z",
            "z_target",

            "vy",
            "vy_target",

            "vz",
            "vz_target",

            "roll",
            "roll_target",

            "pitch",
            "yaw",

            "thrust",

            "joy_roll",
            "joy_pitch",
            "joy_throttle",
            "joy_yaw",

            "dt",

        ]


        self.logger.write_headers(

            actual_header,

            control_header,

        )



    ###################################################
    # Create log messages
    ###################################################

    def create_logs(

        self,

        state,

        command,

        joy_values,

        dt,

    ):


        ###################################################
        # Attitude
        ###################################################

        from tf_transformations import euler_from_quaternion


        roll, pitch, yaw = euler_from_quaternion(

            [

                state.qx,

                state.qy,

                state.qz,

                state.qw,

            ]

        )


        ###################################################
        # Yaw compensated values
        ###################################################

        y_actual = (

            -state.x * math.sin(yaw)

            +

            state.y * math.cos(yaw)

        )


        vy_actual = (

            -state.vx * math.sin(yaw)

            +

            state.vy * math.cos(yaw)

        )



        ###################################################
        # Actual log
        ###################################################

        actual = ActualLog(

            count=self.counter,

            timestamp=state.timestamp,


            qx=state.qx,

            qy=state.qy,

            qz=state.qz,

            qw=state.qw,


            p=state.p,

            q=state.q,

            r=state.r,


            ax=state.ax,

            ay=state.ay,

            az=state.az,


            x=state.x,

            y=state.y,

            z=state.z,


            vx=state.vx,

            vy=state.vy,

            vz=state.vz,


            roll=math.degrees(roll),

            pitch=math.degrees(pitch),

            yaw=math.degrees(yaw),


            y_actual=y_actual,

            vy_actual=vy_actual,

        )


        self.logger.log_actual(

            actual

        )



        ###################################################
        # Control log
        ###################################################

        control = ControlLog(

            count=self.counter,

            timestamp=state.timestamp,


            y=y_actual,

            y_target=self.reference.y,


            z=state.z,

            z_target=self.reference.z,


            vy=vy_actual,

            vy_target=command.vy_desired,


            vz=state.vz,

            vz_target=command.vz_desired,


            roll=math.degrees(roll),

            roll_target=command.roll_deg,


            pitch=math.degrees(pitch),

            yaw=math.degrees(yaw),


            thrust=command.thrust,


            joy_roll=joy_values[0],

            joy_pitch=joy_values[1],

            joy_throttle=joy_values[2],

            joy_yaw=joy_values[3],


            dt=dt,

        )


        self.logger.log_control(

            control

        )



    ###################################################
    # Emergency motor stop
    ###################################################

    def send_motor_stop(self):


        self.get_logger().error(

            "Sending emergency stop"

        )


        msg = Joy()


        msg.buttons = []


        msg.axes = [

            0.0,

            0.0,

            -1.0,

            0.0,

        ]



        for _ in range(20):


            msg.header.stamp = (

                self.get_clock()
                .now()
                .to_msg()

            )


            self.joy_pub.publish(msg)


            time.sleep(0.05)



    ###################################################
    # Shutdown
    ###################################################

    def destroy_node(self):


        self.get_logger().info(

            "Stopping controller"

        )


        self.keyboard.stop()


        self.logger.close()


        super().destroy_node()

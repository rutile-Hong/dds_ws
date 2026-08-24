#!/usr/bin/env python3

"""
joysin.py

Sinusoidal Joy command test with CSV logging.

Behavior:

1. Run joysin.py
      -> ROS2 subscriptions start
      -> CSV logging starts immediately
      -> NO Joy command is sent yet

2. Press 's'
      -> Sinusoidal Joy command starts
      -> Logging continues

3. Ctrl+C
      -> Stop publishing
      -> Flush and close CSV files
"""


import math
import sys
import threading
import termios
import tty

import rclpy

from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor

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


from tf_transformations import euler_from_quaternion


from vehicle_state import VehicleState

from logger import CSVLogger

from messages import (
    ActualLog,
    ControlLog,
)

from config import (
    LoggingConfig,
    QoSConfig,
)


class SinusoidalJoy(Node):

    def __init__(self):

        super().__init__(
            "sinusoidal_joy"
        )

        ###################################################
        # Configuration
        ###################################################

        self.logging_cfg = LoggingConfig()

        self.qos_cfg = QoSConfig()

        ###################################################
        # Vehicle state
        ###################################################

        self.vehicle_state = VehicleState()

        ###################################################
        # Counter
        ###################################################

        self.counter = 0

        ###################################################
        # Sinusoidal Joy settings
        ###################################################

        # Joy throttle center
        self.throttle_center = -0.20

        # Joy sinusoidal amplitude
        self.amplitude = 0.4

        # Frequency [Hz]
        self.frequency = 1.0

        # Joy publishing frequency [Hz]
        self.publish_rate = 100.0

        ###################################################
        # Command state
        ###################################################

        self.command_started = False

        self.command_start_time = None

        ###################################################
        # Current Joy values
        #
        # Before 's' is pressed, these remain NaN because
        # no Joy command has been transmitted.
        ###################################################

        self.joy_roll = float("nan")
        self.joy_pitch = float("nan")
        self.joy_throttle = float("nan")
        self.joy_yaw = float("nan")

        ###################################################
        # QoS
        ###################################################

        sensor_qos = QoSProfile(

            history=HistoryPolicy.KEEP_LAST,

            depth=self.qos_cfg.depth,

            reliability=
            ReliabilityPolicy.BEST_EFFORT,

            durability=
            DurabilityPolicy.VOLATILE,

        )


        command_qos = QoSProfile(

            history=HistoryPolicy.KEEP_LAST,

            depth=10,

            reliability=
            ReliabilityPolicy.RELIABLE,

            durability=
            DurabilityPolicy.VOLATILE,

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
        # Joy publisher
        ###################################################

        self.joy_pub = self.create_publisher(

            Joy,

            "/ap/joy",

            command_qos,

        )


        self.joy_msg = Joy()

        self.joy_msg.buttons = []

        ###################################################
        # CSV Logger
        #
        # Starts immediately when this object is created.
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
        # Timer
        #
        # Timer always runs.
        #
        # BEFORE 's':
        #   log only
        #
        # AFTER 's':
        #   log + publish sine Joy
        ###################################################

        self.timer = self.create_timer(

            1.0 / self.publish_rate,

            self.timer_callback,

        )

        ###################################################
        # Keyboard thread
        ###################################################

        self.keyboard_thread = threading.Thread(

            target=self.keyboard_loop,

            daemon=True,

        )


        self.keyboard_thread.start()

        ###################################################
        # Information
        ###################################################

        self.get_logger().info(
            "======================================"
        )

        self.get_logger().info(
            "Sinusoidal Joy test started"
        )

        self.get_logger().info(
            "CSV LOGGING STARTED"
        )

        self.get_logger().info(
            "No Joy command is being sent yet"
        )

        self.get_logger().info(
            "Press 's' to start sinusoidal Joy"
        )

        self.get_logger().info(
            "Press Ctrl+C to stop"
        )

        self.get_logger().info(
            "======================================"
        )

        self.get_logger().info(

            f"center={self.throttle_center}, "
            f"amplitude={self.amplitude}, "
            f"frequency={self.frequency} Hz, "
            f"publish rate={self.publish_rate} Hz"

        )

    ###################################################
    # ROS callbacks
    ###################################################

    def imu_callback(
        self,
        msg: Imu,
    ):

        self.vehicle_state.update_imu(
            msg
        )


    def pose_callback(
        self,
        msg: PoseStamped,
    ):

        self.vehicle_state.update_pose(
            msg
        )


    def twist_callback(
        self,
        msg: TwistStamped,
    ):

        self.vehicle_state.update_twist(
            msg
        )

    ###################################################
    # Keyboard
    ###################################################

    def keyboard_loop(self):

        """
        Keyboard:

        s = start sinusoidal command
        """

        fd = sys.stdin.fileno()

        old_settings = termios.tcgetattr(
            fd
        )

        try:

            tty.setcbreak(
                fd
            )

            while rclpy.ok():

                key = sys.stdin.read(1)

                #################################################
                # Start sinusoid
                #################################################

                if key.lower() == "s":

                    if not self.command_started:

                        self.command_start_time = (
                            self.get_clock().now()
                        )

                        self.command_started = True

                        self.get_logger().info(
                            "======================================"
                        )

                        self.get_logger().info(
                            "SINUSOIDAL JOY COMMAND STARTED"
                        )

                        self.get_logger().info(

                            f"center="
                            f"{self.throttle_center}, "

                            f"amplitude="
                            f"{self.amplitude}, "

                            f"frequency="
                            f"{self.frequency} Hz"

                        )

                        self.get_logger().info(
                            "======================================"
                        )

                    else:

                        self.get_logger().warn(
                            "Sinusoidal command already running"
                        )

        finally:

            termios.tcsetattr(

                fd,

                termios.TCSADRAIN,

                old_settings,

            )

    ###################################################
    # Timer callback
    ###################################################

    def timer_callback(self):

        now = self.get_clock().now()

        ###################################################
        # Send Joy only after pressing 's'
        ###################################################

        if self.command_started:

            self.publish_sinusoidal_joy(
                now
            )

        ###################################################
        # Logging happens ALWAYS
        ###################################################

        state = self.vehicle_state.copy()

        self.create_logs(
            state
        )

        self.counter += 1

    ###################################################
    # Sinusoidal Joy
    ###################################################

    def publish_sinusoidal_joy(
        self,
        now,
    ):

        ###################################################
        # Time since 's' was pressed
        ###################################################

        t = (

            now
            -
            self.command_start_time

        ).nanoseconds * 1e-9

        ###################################################
        # Sinusoidal signal
        ###################################################

        sine = (

            self.amplitude

            *

            math.sin(

                2.0
                *
                math.pi
                *
                self.frequency
                *
                t

            )

        )


        throttle = (

            self.throttle_center
            +
            sine

        )

        ###################################################
        # Clamp Joy
        ###################################################

        throttle = max(

            -1.0,

            min(

                1.0,

                throttle,

            )

        )

        ###################################################
        # Save current command for logger
        ###################################################

        self.joy_roll = 0.0

        self.joy_pitch = 0.0

        self.joy_throttle = float(
            throttle
        )

        self.joy_yaw = 0.0

        ###################################################
        # Joy message
        ###################################################

        self.joy_msg.header.stamp = (
            now.to_msg()
        )


        self.joy_msg.axes = [

            self.joy_roll,       # Roll

            self.joy_pitch,      # Pitch

            self.joy_throttle,   # Throttle

            self.joy_yaw,        # Yaw

        ]


        self.joy_pub.publish(
            self.joy_msg
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
    # Create CSV log rows
    ###################################################

    def create_logs(
        self,
        state,
    ):

        ###################################################
        # Euler attitude
        ###################################################

        roll, pitch, yaw = euler_from_quaternion(

            [

                state.qx,

                state.qy,

                state.qz,

                state.qw,

            ]

        )

        ###################################################
        # Yaw compensated Y position
        ###################################################

        y_actual = (

            -state.x
            *
            math.sin(yaw)

            +

            state.y
            *
            math.cos(yaw)

        )

        ###################################################
        # Yaw compensated Y velocity
        ###################################################

        vy_actual = (

            -state.vx
            *
            math.sin(yaw)

            +

            state.vy
            *
            math.cos(yaw)

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


            roll=math.degrees(
                roll
            ),

            pitch=math.degrees(
                pitch
            ),

            yaw=math.degrees(
                yaw
            ),


            y_actual=y_actual,

            vy_actual=vy_actual,

        )


        self.logger.log_actual(
            actual
        )

        ###################################################
        # Control / Joy log
        #
        # PID-related fields are NaN because this program
        # is not running your cascade PID.
        ###################################################

        control = ControlLog(

            count=self.counter,

            timestamp=state.timestamp,


            y=y_actual,

            y_target=float("nan"),


            z=state.z,

            z_target=float("nan"),


            vy=vy_actual,

            vy_target=float("nan"),


            vz=state.vz,

            vz_target=float("nan"),


            roll=math.degrees(
                roll
            ),

            roll_target=float("nan"),


            pitch=math.degrees(
                pitch
            ),

            yaw=math.degrees(
                yaw
            ),


            thrust=float("nan"),


            joy_roll=
            self.joy_roll,

            joy_pitch=
            self.joy_pitch,

            joy_throttle=
            self.joy_throttle,

            joy_yaw=
            self.joy_yaw,


            # This test does not use PID dt.
            # 100 Hz nominal timer dt:
            dt=1.0 / self.publish_rate,

        )


        self.logger.log_control(
            control
        )

    ###################################################
    # Shutdown
    ###################################################

    def destroy_node(self):

        self.get_logger().info(
            "Stopping sinusoidal Joy test"
        )

        self.get_logger().info(
            "Closing CSV logger"
        )

        self.logger.close()

        super().destroy_node()


###########################################################
# Main
###########################################################

def main(args=None):

    rclpy.init(
        args=args
    )


    node = SinusoidalJoy()


    executor = MultiThreadedExecutor(
        num_threads=4
    )


    executor.add_node(
        node
    )


    try:

        executor.spin()


    except KeyboardInterrupt:

        node.get_logger().warn(
            "Keyboard interrupt received"
        )


    finally:

        node.destroy_node()

        executor.shutdown()

        rclpy.shutdown()


if __name__ == "__main__":

    main()
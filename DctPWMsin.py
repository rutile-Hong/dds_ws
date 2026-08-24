#!/usr/bin/env python3

import math
import rclpy

from rclpy.node import Node
from ardupilot_msgs.msg import DirectPWM


class SinusoidalPWM(Node):

    def __init__(self):
        super().__init__('sinusoidal_pwm')

        # --------------------------------------------------
        # Publisher
        # --------------------------------------------------
        self.publisher = self.create_publisher(
            DirectPWM,
            '/ap/direct_pwm',
            10
        )

        # --------------------------------------------------
        # Sinusoidal PWM parameters
        # --------------------------------------------------
        self.pwm_center = 1340       # us
        self.amplitude = 100         # us
        self.frequency = 1.0         # Hz

        # Safety limits
        self.pwm_min = 1100
        self.pwm_max = 1800

        # --------------------------------------------------
        # Publishing frequency
        # --------------------------------------------------
        self.publish_rate = 100.0    # Hz

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback
        )

        # Start time
        self.start_time = self.get_clock().now()

        self.get_logger().info(
            f'Sinusoidal PWM started: '
            f'center={self.pwm_center}, '
            f'amplitude={self.amplitude}, '
            f'frequency={self.frequency} Hz'
        )

    def timer_callback(self):

        # --------------------------------------------------
        # Time since start
        # --------------------------------------------------
        now = self.get_clock().now()

        t = (
            now - self.start_time
        ).nanoseconds * 1e-9

        # --------------------------------------------------
        # Generate sinusoidal PWM
        # --------------------------------------------------
        pwm = (
            self.pwm_center
            + self.amplitude
            * math.sin(
                2.0
                * math.pi
                * self.frequency
                * t
            )
        )

        # --------------------------------------------------
        # Limit PWM
        # --------------------------------------------------
        pwm = max(
            self.pwm_min,
            min(self.pwm_max, pwm)
        )

        pwm = int(round(pwm))

        # --------------------------------------------------
        # DDS message
        # --------------------------------------------------
        msg = DirectPWM()

        msg.pwm1 = pwm
        msg.pwm2 = pwm
        msg.enable = True

        self.publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = SinusoidalPWM()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
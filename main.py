#!/usr/bin/env python3

"""
main.py

Entry point for bicopter DDS PID controller.
"""


import rclpy

from rclpy.executors import MultiThreadedExecutor

from node import PIDControlDDSNode



def main(args=None):

    rclpy.init(args=args)


    node = PIDControlDDSNode()


    executor = MultiThreadedExecutor(
        num_threads=4
    )


    executor.add_node(node)


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
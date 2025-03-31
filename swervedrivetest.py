#!/usr/bin/env python3

import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from rclpy import qos


class SwerveController(Node):
    def __init__(self):
        super().__init__("swerve_controller_node")

        # Subscriber cmd_vel
        self.sub_vel = self.create_subscription(
            Twist,
            "cmd_vel",
            self.sub_vel_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )

        # Publisher 2 wheels swerve
        self.pub_wheel = self.create_publisher(
            Float64MultiArray,
            "wheel_controller/commands",
            qos_profile=qos.qos_profile_system_default,
        )

        # Publisher angle wheels swerve
        self.pub_steering = self.create_publisher(
            Float64MultiArray,
            "steering_controller/commands",
            qos_profile=qos.qos_profile_system_default,
        )

        self.wheel = Float64MultiArray()
        self.steering = Float64MultiArray()
        self.max_speed = 6.0 

    def sub_vel_callback(self, cmd_vel):
        """ คำนวณค่าความเร็วและมุมเลี้ยวของล้อ """
        wheel_distance = 0.6  # between wheels (m)

        # speed
        v_left = cmd_vel.linear.x - cmd_vel.angular.z * (wheel_distance / 2)
        v_right = cmd_vel.linear.x + cmd_vel.angular.z * (wheel_distance / 2)

        # angle
        if cmd_vel.linear.x == 0 and cmd_vel.linear.y == 0:
            angle = 0.0  # stay still
        else:
            angle = np.arctan2(cmd_vel.linear.y, cmd_vel.linear.x)

        wheel_data = [v_left, v_right]  # speed
        steering_data = [angle, angle]  # angle

        #  wheels cant more than max_speed 
        max_wheel_speed = max(abs(v_left), abs(v_right))
        if max_wheel_speed > self.max_speed:
            wheel_data = [v / max_wheel_speed * self.max_speed for v in wheel_data]

    
        self.wheel.data = wheel_data
        self.steering.data = steering_data

        self.pub_wheel.publish(self.wheel)
        self.pub_steering.publish(self.steering)

        self.get_logger().info(f"Wheel speeds: L={wheel_data[0]:.2f}, R={wheel_data[1]:.2f} m/s")
        self.get_logger().info(f"Steering angles: {steering_data[0]:.2f} rad")


def main(args=None):
    rclpy.init(args=args)

    node = SwerveController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

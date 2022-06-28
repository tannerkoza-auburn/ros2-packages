import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

import serial
import math
import sys


class RazorImuNode(Node):
    def __init__(self):
        # Node Class Constructor
        super().__init__("razor_imu_node")

        # Constants
        self.DEG_TO_RAD = math.pi / 180
        self.GRAVITY = 9.81 / 1000
        timer_period = 1 / 10000

        # Parameters
        self.declare_parameter("default_port", "/dev/ttyUSB0")
        self.port = (
            self.get_parameter("default_port").get_parameter_value().string_value
        )
        self.declare_parameter("baud_rate", 115200)
        self.baud_rate = (
            self.get_parameter("baud_rate").get_parameter_value().integer_value
        )
        self.declare_parameter("imu_topic", "razor_imu")
        self.imu_topic = (
            self.get_parameter("imu_topic").get_parameter_value().string_value
        )

        # Publisher
        self.razor_imu_publisher = self.create_publisher(Imu, self.imu_topic, 1)

        # Callback
        self.publish_timer = self.create_timer(
            timer_period, self.imu_publisher_callback
        )

        # Initial Serial Read
        self.imu_serial_read()

    def __del__(self):
        self.get_logger().info("Shutting down node! Good-bye!")
        self.destroy_node()
        rclpy.shutdown()

    def imu_serial_read(self):
        self.get_logger().info("Opening %s..." % self.port)
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baud_rate, timeout=1)
            self.get_logger().info("Success! Data is streaming on %s" % self.port)

        except:
            self.get_logger().error(
                "IMU not found on port %s. Did you specify correct port in launch file?"
                % self.port
            )
            sys.exit()

    def imu_publisher_callback(self):
        msg = Imu()
        line = str(self.ser.readline())
        words = line.split(",")

        if len(words) > 2:
            try:

                # Populate Message
                msg.linear_acceleration.x = float(words[2]) * self.GRAVITY
                msg.linear_acceleration.y = float(words[3]) * self.GRAVITY
                msg.linear_acceleration.z = -float(words[4]) * self.GRAVITY

                msg.angular_velocity.x = float(words[5]) * self.DEG_TO_RAD
                msg.angular_velocity.y = float(words[6]) * self.DEG_TO_RAD
                msg.angular_velocity.z = float(words[7]) * self.DEG_TO_RAD

                # Populate Header
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "Open Log Artemis IMU Frame"

                # Publish
                self.razor_imu_publisher.publish(msg)
            except:
                self.get_logger().warning(
                    "Serial Read Error! The following line can't be parsed: \n %s"
                    % line
                )


def main(args=None):
    rclpy.init(args=args)
    node = RazorImuNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from rclpy.qos import ReliabilityPolicy, qos_profile_system_default


class Compressed_RePub(Node):

    def __init__(self):
        super().__init__('compressed_repub')
        self.declare_parameter('reliable_topic', 'reliable_topic')
        self.declare_parameter('best_effort_topic', 'best_effort_topic')
        self.reliable = self.create_subscription(
            CompressedImage,
            self.get_parameter('reliable_topic').get_parameter_value().string_value,
            self.resub_callback,
            10
        )

        best_effort_qos = qos_profile_system_default
        best_effort_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.republisher = self.create_publisher(
            CompressedImage,
            self.get_parameter('best_effort_topic').get_parameter_value().string_value,
            best_effort_qos
        )

    def resub_callback(self, msg):
        self.republisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    compressed_repub = Compressed_RePub()

    rclpy.spin(compressed_repub)

    compressed_repub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
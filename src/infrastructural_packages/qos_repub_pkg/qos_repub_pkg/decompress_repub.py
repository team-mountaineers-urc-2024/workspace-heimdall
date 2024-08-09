import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from rclpy.qos import ReliabilityPolicy, qos_profile_system_default


class Decompress_RePub(Node):

    def __init__(self):
        super().__init__('decompressed_repub')
        self.declare_parameter('reliable_topic', 'reliable_topic')
        self.declare_parameter('best_effort_topic', 'best_effort_topic')
        
        best_effort_qos = qos_profile_system_default
        best_effort_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.reliable = self.create_subscription(
            CompressedImage,
            self.get_parameter('best_effort_topic').get_parameter_value().string_value,
            self.resub_callback,
            best_effort_qos
        )

        self.republisher = self.create_publisher(
            CompressedImage,
            self.get_parameter('reliable_topic').get_parameter_value().string_value,
            10
        )

    def resub_callback(self, msg):
        self.republisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    decompress_repub = Decompress_RePub()

    rclpy.spin(decompress_repub)

    decompress_repub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
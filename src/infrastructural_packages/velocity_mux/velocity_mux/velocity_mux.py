import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String

'''
swaps cmd_vel between autonomy and teleop
'''


class VelocityMux(Node):
    def __init__(self):
        super().__init__('velocity_mux')
        
        self.TELEOP_AXIS = 7

        self.teleop_sub = self.create_subscription(Twist, '/teleop/cmd_vel', self.teleop_callback, 10)
        self.auto_sub = self.create_subscription(Twist, '/auto/cmd_vel', self.auto_callback, 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.led_color_pub = self.create_publisher(String, 'led_color_topic', 10)

        
        self.teleop_mode = True  # Default to teleop mode
        self.zero_twist = Twist()

    def teleop_callback(self, msg):
        if self.teleop_mode:
            self.cmd_pub.publish(msg)
        self.teleop_twist = msg

    def auto_callback(self, msg):
        if not self.teleop_mode:
            self.cmd_pub.publish(msg)
        self.auto_twist = msg

    def joy_callback(self, msg):
        # Assuming buttons[0] is the down button and buttons[3] is the up button (D-pad down and up)
        if msg.axes[self.TELEOP_AXIS] < 0 and not self.teleop_mode:
            self.teleop_mode = True
            self.get_logger().info('Switched to teleop mode')
            self.led_color_pub.publish(String(data="teleop"))
            self.cmd_pub.publish(self.zero_twist)
            
        elif msg.axes[self.TELEOP_AXIS] > 0 and self.teleop_mode:
            self.teleop_mode = False
            self.get_logger().info('Switched to auto mode')
            self.led_color_pub.publish(String(data="autonomous"))
            self.cmd_pub.publish(self.zero_twist)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


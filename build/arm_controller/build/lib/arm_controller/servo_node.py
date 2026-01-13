import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
# We use a try-except block here so the node doesn't crash if hardware isn't plugged in yet
try:
    from adafruit_servokit import ServoKit
except ImportError:
    ServoKit = None

class ArmServoNode(Node):
    def __init__(self):
        super().__init__('arm_servo_node')
        
        self.kit = None
        
        # Initialize PCA9685 (16 channels)
        try:
            if ServoKit:
                # address=0x40 is standard. On Pi 5, ensures correct I2C bus.
                self.kit = ServoKit(channels=16)
                self.get_logger().info('PCA9685 Servo Driver Connected!')
            else:
                self.get_logger().warn('Adafruit library not found.')
        except Exception as e:
            self.get_logger().error(f'Could not connect to servo board: {e}')
            self.get_logger().error('Check if I2C is enabled in raspi-config and wires are secure.')

        # Subscribe to "joint_commands"
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'joint_commands',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        angles = msg.data
        if len(angles) != 6:
            self.get_logger().warn(f'Expected 6 angles, got {len(angles)}')
            return

        self.get_logger().info(f'Moving to: {angles}')
        
        if self.kit:
            for i in range(6):
                target_angle = angles[i]
                # Safety clamp for MG996R (0 to 180 usually safest to start)
                if target_angle < 0: target_angle = 0
                if target_angle > 180: target_angle = 180 
                
                # Move servo (Channels 0-5 on the board)
                self.kit.servo[i].angle = target_angle

def main(args=None):
    rclpy.init(args=args)
    node = ArmServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import RPi.GPIO as GPIO
import time

class CmdVelRelayBlinker(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay_blinker')

        # Setup GPIO
        self.relay_pin = 21
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.relay_pin, GPIO.OUT)
        GPIO.output(self.relay_pin, GPIO.LOW)

        self.get_logger().info(f'[INIT] Relay GPIO setup complete on pin {self.relay_pin}')

        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            TwistStamped,
            '/diff_cont/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('[INIT] Subscribed to /diff_cont/cmd_vel')

    def cmd_vel_callback(self, msg):
        # Access the Twist message inside TwistStamped
        linear = msg.twist.linear.x
        angular = msg.twist.angular.z

        self.get_logger().info(f'[CALLBACK] Received cmd_vel: linear.x = {linear}, angular.z = {angular}')

        # Check if there's movement
        if abs(linear) > 0.01 or abs(angular) > 0.01:
            self.get_logger().info('[CALLBACK] Non-zero velocity detected. Triggering relay blink...')
            self.blink_relay()
        else:
            self.get_logger().info('[CALLBACK] Zero velocity command. No relay action.')

    def blink_relay(self):
        try:
            self.get_logger().info('[RELAY] Turning relay ON...')
            GPIO.output(self.relay_pin, GPIO.HIGH)
            time.sleep(0.5)

            self.get_logger().info('[RELAY] Turning relay OFF...')
            GPIO.output(self.relay_pin, GPIO.LOW)
            time.sleep(0.5)

            self.get_logger().info('[RELAY] Blink complete.')

        except Exception as e:
            self.get_logger().error(f'[ERROR] Failed to blink relay: {e}')

    def destroy_node(self):
        GPIO.cleanup()
        self.get_logger().info('[CLEANUP] GPIO cleaned up.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelayBlinker()
    node.get_logger().info('[MAIN] Node started. Waiting for cmd_vel messages...')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[SHUTDOWN] Keyboard interrupt received. Exiting...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print('[MAIN] Shutdown complete.')

if __name__ == '__main__':
    main()

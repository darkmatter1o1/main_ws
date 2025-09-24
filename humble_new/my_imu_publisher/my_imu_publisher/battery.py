import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from collections import deque


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')

        # Set up I2C and ADS1115
        i2c = busio.I2C(board.SCL, board.SDA)
        self.ads = ADS.ADS1115(i2c)
        self.ads.gain = 1  # ±4.096V
        self.chan = AnalogIn(self.ads, ADS.P0)

        # Set up ROS publisher
        self.publisher_ = self.create_publisher(Float32, '/battery_level', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Voltage scaling & limits
        self.scaling_factor = 5.10
        self.max_voltage = 13
        self.min_voltage = 12.65

        # Filtering parameters
        self.readings = deque(maxlen=20)  # Moving average buffer
        self.filtered_voltage = None      # EMA value
        self.ema_alpha = 0.3               # Smoothing factor (lower = smoother)

    def timer_callback(self):
        try:
            # Raw reading scaled to battery voltage
            raw_voltage = self.chan.voltage * self.scaling_factor

            # --- Moving average filter ---
            self.readings.append(raw_voltage)
            avg_voltage = sum(self.readings) / len(self.readings)

            # --- Exponential moving average ---
            if self.filtered_voltage is None:
                self.filtered_voltage = avg_voltage
            else:
                self.filtered_voltage = (
                    self.ema_alpha * avg_voltage
                    + (1 - self.ema_alpha) * self.filtered_voltage
                )

            # Clamp voltage
            stable_voltage = max(min(self.filtered_voltage, self.max_voltage), self.min_voltage)

            # Convert to percentage
            percentage = ((stable_voltage - self.min_voltage) / (self.max_voltage - self.min_voltage)) * 100
            percentage = max(0.0, min(percentage, 100.0))

            # Publish
            msg = Float32()
            msg.data = percentage
            self.publisher_.publish(msg)

            # Log
            self.get_logger().info(f"Battery: {percentage:.1f}% ({stable_voltage:.2f}V)")

        except Exception as e:
            self.get_logger().error(f"Error reading battery voltage: {e}")


def main(args=None):
    rclpy.init(args=args)
    battery_monitor = BatteryMonitor()
    try:
        rclpy.spin(battery_monitor)
    except KeyboardInterrupt:
        pass
    battery_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

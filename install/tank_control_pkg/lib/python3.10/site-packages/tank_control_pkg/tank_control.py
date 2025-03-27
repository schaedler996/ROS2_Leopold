import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import smbus2
import yaml
from time import time

class I2CController:
    def __init__(self, address, bus_number=1):
        self.address = address
        self.bus = smbus2.SMBus(bus_number)
        self.reset_values()
        self.error_count = 0  # Initialisiere den Fehlerzähler

    def reset_values(self):
        self.motor1_speed = 0
        self.motor2_speed = 0
        self.servo1_pos = 90
        self.servo2_pos = 90
        self.stepper_speed = 0

    def send_all_commands(self):
        motor1_speed = self.motor1_speed & 0xFF
        motor2_speed = self.motor2_speed & 0xFF
        stepper_speed = self.stepper_speed & 0xFF

        data = [
            motor1_speed,
            motor2_speed,
            self.servo1_pos,
            self.servo2_pos,
            stepper_speed
        ]

        try:
            self.bus.write_i2c_block_data(self.address, 0, data)
            # Wenn das Senden erfolgreich ist, setze den Fehlerzähler zurück
            self.error_count = 0
        except Exception as e:
            self.error_count += 1
            print(f"Fehler beim Senden der Daten (Versuch {self.error_count}): {e}")

    def set_motor_speeds(self, motor1, motor2):
        if -128 <= motor1 <= 127 and -128 <= motor2 <= 127:
            self.motor1_speed = int(motor1)
            self.motor2_speed = int(motor2)

    def set_servo_positions(self, servo1, servo2):
        if 0 <= servo1 <= 180 and 0 <= servo2 <= 180:
            self.servo1_pos = int(servo1)
            self.servo2_pos = int(servo2)

    def set_stepper_speed(self, speed):
        if -128 <= speed <= 127:
            self.stepper_speed = int(speed)


class TankControl(Node):

    def subscription_callback(self, msg):
        # Berechne Geschwindigkeiten basierend auf der Twist-Nachricht
        self.linear_x = self.apply_exponential_curve(msg.linear.x, exponent=self.expo_linear)
        self.angular_z = self.apply_exponential_curve(msg.angular.z, exponent=self.expo_angular)

        corrected_linear_x = self.linear_x * self.linear_speed_adjusted
        angular_speed_amplified = self.angular_z * self.angular_amp
        
        # Maximalgeschwindigkeit
        max_speed = 127

        # Berechnungen
    
        left_speed = self.map_range(corrected_linear_x - angular_speed_amplified, -1.0, 1.0, -max_speed, max_speed)
        right_speed = self.map_range(corrected_linear_x + angular_speed_amplified, -1.0, 1.0, -max_speed, max_speed)
        turret_speed = self.map_range(self.joy_axes[2], -0.99, 0.99, -127, 127)
        
        # Begrenze die Geschwindigkeiten, um sicherzustellen, dass sie innerhalb von [-127, 127] bleiben
        left_speed = max(min(left_speed, max_speed), -max_speed)
        right_speed = max(min(right_speed, max_speed), -max_speed)
        turret_speed = max(min(turret_speed, max_speed), -max_speed)

        # Zeige Achse und Geschwindigkeiten an
        print(f"Left Speed: {int(left_speed)}, Right Speed: {int(right_speed)}, Turret Speed: {int(turret_speed)}")

        self.i2c_controller.set_motor_speeds(left_speed, right_speed)
        self.i2c_controller.set_stepper_speed(turret_speed)
        self.last_msg_time = time()

    def joy_callback(self, joy_msg):
        # X von Joy-Message
        self.joy_x = joy_msg.axes[0]  # Erster Achsenwert
        self.joy_axes = joy_msg.axes  # Ganze Liste
    
    def __init__(self):
        super().__init__('tank_control')
        self.i2c_controller = I2CController(0x08)

        try:
            self.config = self.load_yaml_config("/home/jetson/ros2_ws/src/params_pkg/params/robot_params.yaml")
        except Exception as e:
            self.get_logger().error(f'Critical error: {e}')
            raise Exception(f'Failed to load configuration: {e}')

        self.left_track_correction = self.config['robot_parameters']['track_correction_factor']['left_track']
        self.right_track_correction = self.config['robot_parameters']['track_correction_factor']['right_track']
        self.max_linear_speed = self.config['robot_parameters']['max_linear_speed']
        self.linear_speed_adjusted = self.config['robot_parameters']['track_correction_factor']['linear_speed_adjusted']
        self.expo_linear = self.config['robot_parameters']['expo_linear']
        self.expo_angular = self.config['robot_parameters']['expo_angular']
        self.angular_amp = self.config['robot_parameters']['angular_speed_amplification_factor']
        
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.subscription_callback, 10)
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.last_msg_time = time()

    def load_yaml_config(self, file_path):
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)

    def map_range(self, value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def timer_callback(self):
        elapsed_time = time() - self.last_msg_time
        # Always resend commands every timer tick
        self.i2c_controller.send_all_commands()
        # Optionally stop motors if no message after a certain period
        if elapsed_time >= 0.1:
            self.i2c_controller.set_motor_speeds(0, 0)
            self.i2c_controller.set_stepper_speed(0)

    @staticmethod
    def apply_exponential_curve(value, exponent):
        sign = 1 if value >= 0 else -1
        return sign * (abs(value) ** exponent)

def main(args=None):
    rclpy.init(args=args)
    tank_control = TankControl()
    rclpy.spin(tank_control)
    tank_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
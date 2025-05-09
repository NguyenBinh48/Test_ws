import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO   #pip install RPi.GPIO
import time

class FollowAruco2(Node):
    def __init__(self):
        super().__init__('follow_aruco_irl')
        self.get_logger().info('Following ArUco marker...')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/aruco_position',
            self.listener_callback,
            10
        )

        # Declare parameters
        self.declare_parameter('rcv_timeout_secs', 1.0)
        self.declare_parameter('angular_chase_multiplier', 0.01)
        self.declare_parameter('forward_chase_speed', 0.2)
        self.declare_parameter('search_angular_speed', 0.5)
        self.declare_parameter('min_distance_thresh', 30.0)
        self.declare_parameter('stop_distance_thresh', 27.0)
        self.declare_parameter('filter_value', 0.9)

        # Get parameters
        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
        self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
        self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
        self.min_distance_thresh = self.get_parameter('min_distance_thresh').get_parameter_value().double_value
        self.stop_distance_thresh = self.get_parameter('stop_distance_thresh').get_parameter_value().double_value
        self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.ENA = 18
        self.IN1 = 17
        self.IN2 = 27
        self.ENB = 12
        self.IN3 = 22
        self.IN4 = 23
        GPIO.setup(self.ENA, GPIO.OUT)
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        GPIO.setup(self.ENB, GPIO.OUT)
        GPIO.setup(self.IN3, GPIO.OUT)
        GPIO.setup(self.IN4, GPIO.OUT)
        self.pwm_a = GPIO.PWM(self.ENA, 100)
        self.pwm_b = GPIO.PWM(self.ENB, 100)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

        # Initialize state
        self.target_x = 0.0  # Lateral offset (cm)
        self.target_z = 0.0  # Distance (cm)
        self.lastrcvtime = time.time() - 10000

        # Control loop timer
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def move_straight(self, speed_percent):
        """Move robot straight (0-100 speed)."""
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_a.ChangeDutyCycle(speed_percent)
        self.pwm_b.ChangeDutyCycle(speed_percent)

    def rotate(self, angular_z):
        """Rotate robot based on angular_z (rad/s). Positive: CCW, Negative: CW."""
        # Map angular_z (rad/s) to PWM difference (0-100%)
        # Assume max angular speed of 1 rad/s maps to 100% speed difference
        speed_diff = min(abs(angular_z) * 100, 100)  # Scale to 0-100%
        base_speed = 50  # Base speed for rotation
        if angular_z > 0:  # CCW: Left slower, Right faster
            left_speed = max(0, base_speed - speed_diff)
            right_speed = min(100, base_speed + speed_diff)
            GPIO.output(self.IN1, GPIO.HIGH)
            GPIO.output(self.IN2, GPIO.LOW)
            GPIO.output(self.IN3, GPIO.HIGH)
            GPIO.output(self.IN4, GPIO.LOW)
        else:  # CW: Left faster, Right slower
            left_speed = min(100, base_speed + speed_diff)
            right_speed = max(0, base_speed - speed_diff)
            GPIO.output(self.IN1, GPIO.HIGH)
            GPIO.output(self.IN2, GPIO.LOW)
            GPIO.output(self.IN3, GPIO.HIGH)
            GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_a.ChangeDutyCycle(left_speed)
        self.pwm_b.ChangeDutyCycle(right_speed)

    def stop(self):
        """Stop both motors."""
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)

    def timer_callback(self):
        if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
            self.get_logger().info(f'Tracking ArUco: x={self.target_x:.1f} cm, z={self.target_z:.1f} cm')
            if self.target_z <= self.stop_distance_thresh:
                # Stop all movement when too close
                self.stop()
                self.get_logger().info('ArUco marker reached, stopping.')
            else:
                # Normal tracking behavior
                linear_x = self.forward_chase_speed if self.target_z > self.min_distance_thresh else 0.0
                angular_z = -self.angular_chase_multiplier * self.target_x
                if linear_x > 0:
                    self.move_straight(linear_x * 100)  # Scale 0-1 to 0-100%
                else:
                    self.stop()
                if abs(angular_z) > 0.1:  # Apply rotation only if significant
                    self.rotate(angular_z)
        else:
            self.get_logger().info('ArUco marker lost')
            self.rotate(self.search_angular_speed)  # Rotate to search

    def listener_callback(self, msg):
        try:
            x, y, z = msg.data  # x, y, z in cm
            f = self.filter_value
            self.target_x = self.target_x * f + x * (1 - f)
            self.target_z = self.target_z * f + z * (1 - f)
            self.lastrcvtime = time.time()
        except Exception as e:
            self.get_logger().error(f'Error processing position: {e}')

    def destroy_node(self):
        self.stop()
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    follow_aruco_irl = FollowAruco2()
    try:
        rclpy.spin(follow_aruco_irl)
    except KeyboardInterrupt:
        pass
    finally:
        follow_aruco_irl.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
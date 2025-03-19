import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np

class Goto_Start(Node):

    def __init__(self, trajectory_file):
        super().__init__('goto_start')
        self.publisher_ = self.create_publisher(Twist, '/a200_0656/twist_marker_server/cmd_vel', 10)
        self.subscriber = self.create_subscription(Odometry, '/zed/zed_node/odom', self.odom_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.timer = self.create_timer(0.1, self.motion)  # 10 Hz control loop

        # State variables
        self.x_position = 0.0
        self.y_position = 0.0
        self.yaw = 0.0
        self.state = "initializing"

        # Command variables
        self.cmd = Twist()

        # Target variables
        self.x_target = 0.0
        self.y_target = 0.0
        self.yaw_target = 0.0

        # Configuration parameters
        self.declare_parameter('max_linear_speed', 0.2)
        self.declare_parameter('max_angular_speed', 0.2)
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('angle_tolerance', 0.05)

        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value

        # Load trajectory and set target position
        self.load_trajectory(trajectory_file)

    def load_trajectory(self, file_path):
        try:
            self.trajectory = np.genfromtxt(file_path, delimiter=',', skip_header=1)
            if self.trajectory.shape[0] == 0:
                raise ValueError("Trajectory file is empty")
            self.x_target, self.y_target, self.yaw_target= self.trajectory[0, 0], self.trajectory[0, 1], self.trajectory[0, 2]
            self.get_logger().info(f'Loaded trajectory. Start position: ({self.x_target}, {self.y_target}, {self.yaw_target})')
        except (IOError, ValueError) as e:
            self.get_logger().error(f'Failed to load trajectory: {str(e)}')
            rclpy.shutdown()

    def euler_from_quaternion(self, quaternion):
    #function to convert quarternion to euler angles
        x, y, z, w = quaternion
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def odom_callback(self, msg):
        self.x_position = msg.pose.pose.position.x
        self.y_position = msg.pose.pose.position.y
        
        #get orientation and convert to euler angle (theta)
        orientation_q = msg.pose.pose.orientation
        _, _, self.yaw = self.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    
    def calculate_linear_velocity(self, error):
        Kp = 0.5  # Proportional gain
        return np.clip(Kp * error, -self.max_linear_speed, self.max_linear_speed)

    def calculate_angular_velocity(self, error):
        Kp = 1.0  # Proportional gain
        return np.clip(Kp * error, -self.max_angular_speed, self.max_angular_speed)

    def motion(self):
        self.get_logger().info(f'Current position: x: {self.x_position:.2f}, y: {self.y_position:.2f}, yaw: {self.yaw:.2f}, state: {self.state}')

        if self.state == "initializing":
            self.state = "move_to_position"
            self.get_logger().info(f'Initialized. Moving to start position: ({self.x_target:.2f}, {self.y_target:.2f})')

        elif self.state == "move_to_position":
            dx = self.x_target - self.x_position
            dy = self.y_target - self.y_position
            distance = np.sqrt(dx**2 + dy**2)

            if distance > self.position_tolerance:
                angle_to_target = np.arctan2(dy, dx)
                angle_diff = self.normalize_angle(angle_to_target - self.yaw)

                if abs(angle_diff) > self.angle_tolerance:
                    self.cmd.linear.x = 0.0
                    self.cmd.angular.z = self.calculate_angular_velocity(angle_diff)
                    self.get_logger().info(f'Rotating to face target. Angle difference: {angle_diff:.2f}')
                else:
                    self.cmd.linear.x = self.calculate_linear_velocity(distance)
                    self.cmd.angular.z = 0.0
                    self.get_logger().info(f'Moving towards target. Distance: {distance:.2f}')
            else:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.state = "adjust_theta"
                self.get_logger().info('Reached target position. Adjusting theta.')

        elif self.state == "adjust_theta":
            angle_diff = self.normalize_angle(self.yaw_target - self.yaw)

            if abs(angle_diff) > self.angle_tolerance:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = self.calculate_angular_velocity(angle_diff)
                self.get_logger().info(f'Adjusting theta. Angle difference: {angle_diff:.2f}')
            else:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.state = "complete"
                self.get_logger().info('Reached target position and orientation. Task complete.')

        elif self.state == "complete":
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.get_logger().info('Task completed. Shutting down...')
            rclpy.shutdown()

        self.publisher_.publish(self.cmd)

    def normalize_angle(self, angle):
        
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    
    trajectory_file = '/home/administrator/nmpc_ws/data/trajectories/recorded_odometry.csv'
    goto = Goto_Start(trajectory_file)
    rclpy.spin(goto)
    
    goto.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
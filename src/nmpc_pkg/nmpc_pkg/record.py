import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import os
import csv
import numpy as np
import matplotlib.pyplot as plt

# Global variables
current_x = 0.0
current_y = 0.0
current_theta = 0.0
trajectory_data = []

def euler_from_quaternion(quaternion):
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

def odom_callback(msg):
    global current_x, current_y, current_theta, trajectory_data
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    _, _, current_theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    trajectory_data.append((current_x, current_y, current_theta))

def record_trajectory():
    rclpy.init()
    node = Node('trajectory_recorder')

    # Subscribe to the filtered odometry topic
    node.create_subscription(Odometry, '/zed/zed_node/odom', odom_callback, 10)

    # Create folder for CSV file storage
    folder_path = os.path.join(os.path.expanduser('~'), 'nmpc_ws', 'data', 'trajectories')
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    csv_file_path = os.path.join(folder_path, 'recorded_odometry.csv')

    with open(csv_file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['X (m)', 'Y (m)', 'Theta (rad)'])
        print("Recording started. Press Ctrl+C to stop.")

        try:
            while rclpy.ok():
                rclpy.spin_once(node)
                writer.writerow([current_x, current_y, current_theta])
        except KeyboardInterrupt:
            print("Recording stopped.")
            plot_trajectory()
        finally:
            node.destroy_node()
            # Check if ROS is still running before attempting shutdown
            if rclpy.ok():
                rclpy.shutdown()
            else:
                print("ROS2 context already shut down.")

def plot_trajectory():
    global trajectory_data
    x_vals, y_vals = zip(*[(x, y) for x, y, _ in trajectory_data])
    plt.figure()
    plt.plot(x_vals, y_vals, marker='o', linestyle='None')
    plt.title('Robot Trajectory')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.axis('equal')
    plt.grid()
    folder_path = os.path.join(os.path.expanduser('~'), 'nmpc_ws', 'data', 'trajectories')
    plot_file_path = os.path.join(folder_path, 'recorded_odometry_plot.png')
    plt.savefig(plot_file_path)
    plt.show()

def main():
    record_trajectory()

if __name__ == '__main__':
    main()

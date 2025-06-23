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
prev_x = 0.0
prev_y = 0.0
prev_theta = 0.0
offset_x = 0.0
offset_y = 0.0
offset_theta = 0.0
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
    global prev_x, prev_y, prev_theta
    global offset_x, offset_y, offset_theta

    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    _, _, current_theta = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                                 msg.pose.pose.orientation.y, 
                                                 msg.pose.pose.orientation.z, 
                                                 msg.pose.pose.orientation.w])
    
    if current_x == 0 and current_y == 0 and current_theta == 0:
        offset_x = prev_x
        offset_y = prev_y
        offset_theta = prev_theta

        print("Offset: ", offset_x, offset_y, offset_theta)

    x = current_x + offset_x
    y = current_y + offset_y
    theta = current_theta + offset_theta
    
    trajectory_data.append((x, y, theta))

    prev_x = current_x
    prev_y = current_y
    prev_theta = current_theta


def record_trajectory():
    rclpy.init()
    node = Node('trajectory_recorder')

    # Subscribe to the filtered odometry topic
    node.create_subscription(Odometry,'/vtr/odometry', odom_callback, 10) # '/camera_odom'

    # Create folder for CSV file storage
    # folder_path = os.path.join(os.path.expanduser('~'), 'code', 'nmpc_ws', 'data', 'trajectories')
    folder_path = "/home/adam/Desktop/MPC/data/trajectories"
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    csv_file_path = os.path.join(folder_path, 'new_odometry.csv')

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
    folder_path = "/home/adam/Desktop/MPC/data/trajectories"
    # folder_path = os.path.join(os.path.expanduser('~'), 'code', 'nmpc_ws', 'data', 'trajectories')
    plot_file_path = os.path.join(folder_path, 'new_odometry_plot.png')
    plt.savefig(plot_file_path)
    plt.show()

def main():
    record_trajectory()

if __name__ == '__main__':
    main()

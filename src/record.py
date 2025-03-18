import rclpy
import csv
import math
import os
import numpy as np
import matplotlib.pyplot as plt
from rclpy.node import Node
from nav_msgs.msg import Odometry

#global variables to store the odometry data
current_x = 0.0
current_y = 0.0
current_theta = 0.0
trajectory_data = []

def euler_from_quaternion(quaternion):
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

def odom_callback(msg):
    # Callback to handle odometry messages
    global current_x, current_y, current_theta, trajectory_data
    
    # Get position (x, y) from odometry message
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    
    #get orientation and convert to euler angle (theta)
    orientation_q = msg.pose.pose.orientation
    _, _, current_theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    # Append current data to the trajectory
    trajectory_data.append((current_x, current_y, current_theta))

def record_trajectory():
    rclpy.init()
    node = Node('trajectory_recorder')

    # Subscribe to the /odometry topic to receive odometry data
    node.create_subscription(Odometry, '/odometry', odom_callback, 10)

    # Create the folder inside to store the CSV file
    folder_path = os.path.join(os.path.expanduser('~'), 'data', 'nicola_watt', 'trajectories')
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    # Path to the CSV file
    csv_file_path = os.path.join(folder_path, 'recorded_odometry.csv')

    # Open CSV file
    with open(csv_file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        #header row
        writer.writerow(['X (m)', 'Y (m)', 'Theta (rad)'])

        print("Recording started. Press Ctrl+C to stop.")
        
        # Run the node and write data as it comes in
        try:
            while rclpy.ok():
                # Spin once to check for new messages
                rclpy.spin_once(node)

                # Write the current odometry data to the CSV file if it's been updated
                writer.writerow([current_x, current_y, current_theta])

        except KeyboardInterrupt:
            print("Recording stopped.")
            # Plot the trajectory
            plot_trajectory()
        finally:
            # Cleanup
            node.destroy_node()
            rclpy.shutdown()

def plot_trajectory():
    #generate a plot of the recorded points
    global trajectory_data
    x_vals, y_vals = zip(*[(x, y) for x, y, _ in trajectory_data])

    plt.figure()
    plt.plot(x_vals, y_vals, marker='o', linestyle='None')
    plt.title('Robot Trajectory')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.axis('equal')
    plt.grid()

    # Save the plot as PNG
    folder_path = os.path.join(os.path.expanduser('~'), 'data', 'nicola_watt', 'trajectories')
    plot_file_path = os.path.join(folder_path, 'recorded_odometry_plot.png')
    plt.savefig(plot_file_path)
    plt.show()

if __name__ == '__main__':
    record_trajectory()
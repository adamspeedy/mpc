#!/usr/bin/env python3
import numpy as np
import csv
import socket
import json
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from .controller_class import Controller  

class NMPCNode(Node):
    PLOTTER_ADDRESS = ('196.24.139.89', 12345)     #hardcoded ip address for external plotter
    PATH_TYPE = 'repeat'                            #path-following behaviour options: 'stop' or 'repeat'

    def __init__(self):
        super().__init__('nmpc_controller_node')

        self._init_parameters()
        self._init_state_variables()
        self._init_communication()
        self._init_controller()
        
    def _init_parameters(self):
        #load parameters
        self.get_logger().info("Loading parameters...")

        self.declare_parameter('rate', 10)
        self.declare_parameter('trajectory_file', '/home/administrator/nmpc_ws/data/trajectories/recorded_odometry.csv')
        self.declare_parameter('min_v', -1.0)
        self.declare_parameter('max_v', 1.0)
        self.declare_parameter('min_w', -1.5)
        self.declare_parameter('max_w', 1.5)
        
        #retrieve parameter values
        self.rate = self.get_parameter('rate').value
        self.csv_file = self.get_parameter('trajectory_file').value

    def _init_state_variables(self):
        #initialise state variables
        self.current_state = None

        #initialise previous control
        self.previous_control = np.zeros(2)
        
        #initialise data storage for robot trajectory
        self.actual_trajectory = []
        
        #load reference trajectory from csv file
        self.reference_trajectory = self.load_trajectory()
          
    def _init_communication(self):
        #initialise ROS publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/a200_0656/twist_marker_server/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/zed/zed_node/odom', self.odom_callback, 10)

        #wait for initial position
        self.initial_position_received = False
        self.initial_position = None
        while not self.initial_position_received:
            rclpy.spin_once(self)
    
        #create udp socket to send trajectory data to external plotter
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    def _init_controller(self):
        #get controller parameters
        min_v = self.get_parameter('min_v').value
        max_v = self.get_parameter('max_v').value
        min_w = self.get_parameter('min_w').value
        max_w = self.get_parameter('max_w').value
        
        #initialise nmpc controller with initial position and parameters
        self.controller = Controller(min_v, max_v, min_w, max_w, T=1.0/self.rate)
        self.trajectory_index = None
        
        #timer for control loop
        self.timer = self.create_timer(1.0/self.rate, self.control_loop)

        #path-following behaviour options: 'stop' or 'repeat'
        self.path_type = self.PATH_TYPE
        self.stop = False 

    def load_trajectory(self):
    #function to load reference trajectory from csv file
        reference_trajectory = []
        with open(self.csv_file, 'r') as file:
            csv_reader = csv.reader(file)
            next(csv_reader)
            for row in csv_reader:
                x, y, theta = map(float, row)
                reference_trajectory.append([x, y, theta])
        return np.array(reference_trajectory)

    def odom_callback(self, msg):
    #function to process odometry messages
        #extract current position
        self.x_position = msg.pose.pose.position.x
        self.y_position = msg.pose.pose.position.y

        if self.x_position == 0.0 and self.y_position == 0.0:
            return
        
        #extract current orientation and convert to euler angles
        orientation_q = msg.pose.pose.orientation
        _, _, self.yaw = self.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        #update current state
        self.current_state = np.array([self.x_position, self.y_position, self.yaw])

        #set initial position 
        if not self.initial_position_received:
            self.initial_position = self.current_state
            self.initial_position_received = True
            
        self.odom_received = True

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

    def find_closest_point_index(self, current_state):
        #extract the position and yaw from the current state
        pos = current_state[:2]
        yaw = current_state[2]

        #euclidean distances to each reference point
        distances = np.linalg.norm(self.reference_trajectory[:, :2] - pos, axis=1)
    
        #unwrap reference orientations to avoid discontinuities
        ref_angles = np.unwrap(self.reference_trajectory[:, 2])
    
        #orientation difference
        angle_diffs = np.abs(np.arctan2(np.sin(ref_angles - yaw), np.cos(ref_angles - yaw)))
    
        #weight to balance distance and orientation difference
        weight = 0.2
    
        #combined cost
        cost = distances + weight * angle_diffs
    
        #return index
        return np.argmin(cost)


    def reference_trajectory_N(self):
    #function to retrieve next N steps of the reference trajectory

        #get index of closest point
        closest_index = self.find_closest_point_index(self.current_state)

        N = self.controller.N
        total_points = len(self.reference_trajectory)
        
        #create an array to hold N+1 points
        ref_traj = np.zeros((N+1, 3))
        
        for i in range(N+1):
            index = (closest_index + i) % total_points
            ref_traj[i] = self.reference_trajectory[index]

        #unwrap angles in reference trajectory
        ref_traj[:, 2] = np.unwrap(ref_traj[:, 2])
        
        #if end of trajectory is reached and stop is requested
        if closest_index + N >= total_points and self.path_type == 'stop':
            self.stop = True
        
        return ref_traj

    def send_data(self):
    #function to send trajectory data to external plotter

        closest_index = self.find_closest_point_index(self.current_state)
        closest_point = self.reference_trajectory[closest_index]

        trajectory_data ={
            'actual_x' : float(self.current_state[0]),
            'actual_y' : float(self.current_state[1]),
            'forecast_x': self.controller.next_states[:, 0].tolist() if hasattr(self.controller, 'next_states') else [],
            'forecast_y': self.controller.next_states[:, 1].tolist() if hasattr(self.controller, 'next_states') else [], 
            'closest_x' : float(closest_point[0]),
            'closest_y' : float(closest_point[1]),
            'optimisation_time' : float(self.time_taken)
        }

        #send data as json encoded udp
        self.socket.sendto(json.dumps(trajectory_data).encode(), self.PLOTTER_ADDRESS)

    def unwrap_current_state(self, current_state, ref_trajectory):
        #unwrap current yaw relative to reference trajectory
        ref_angle = ref_trajectory[0, 2]
        current_angle = current_state[2]
    
        #calculate difference
        diff = current_angle - ref_angle
    
        #adjust for wrap-around
        if abs(diff) > np.pi:
            if diff > 0:
                current_angle -= 2 * np.pi
            else:
                current_angle += 2 * np.pi
    
        return np.array([current_state[0], current_state[1], current_angle])

    def control_loop(self):
    #control loop runs periodically
        if not self.odom_received:
            #wait for odometry data
            self.get_logger().info('Waiting for initial odometry data...')
            return
        
        else:
            #self.get_logger().info('Current position:')
            #print(self.current_state)
            
            #get reference trajectory for next N steps
            ref_trajectory = self.reference_trajectory_N()

            #create array of reference controls based on previous controls (ie minimise change in control)
            ref_controls = np.tile(self.previous_control, (self.controller.N, 1))

            #unwrap current state
            current_state_unwrapped = self.unwrap_current_state(self.current_state, ref_trajectory)
        
            #log start time
            start_time = time.time()

            #solve nmpc problem
            self.optimal_control = self.controller.solve(current_state_unwrapped, ref_trajectory, ref_controls)

            #log end time and compute time taken for optimisation calculation
            end_time = time.time()
            self.time_taken = end_time - start_time
        
            #create and publish velocity command
            cmd_vel_msg = Twist()
            if self.stop == True:
                #stop
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
            else:
                #apply optimal control inputs
                cmd_vel_msg.linear.x = float(self.optimal_control[0])
                cmd_vel_msg.angular.z = float(self.optimal_control[1])
    
            self.cmd_vel_pub.publish(cmd_vel_msg)

            #send trajectory data for plotting
            self.send_data()
        
def destroy_node(self):
#function to close udp socket and destroy controller node
        self.socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    nmpc_node = NMPCNode()
    try:
        rclpy.spin(nmpc_node)
    except KeyboardInterrupt:
        pass
    nmpc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import numpy as np
import socket
import json
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from .controller_class import Controller  

class NaviganNMPCNode(Node):
   PLOTTER_ADDRESS = ('196.24.161.117', 12345)
   PATH_TYPE = 'repeat' 

   def __init__(self):
      super().__init__('navigan_nmpc_controller_node')   
        
      self._init_parameters()
      self._init_state_variables()
      self._init_communication()
      self._init_controller()

   def _init_parameters(self):
      #load parameters
      self.get_logger().info("Loading parameters...")

      self.declare_parameter('rate', 10)
      self.declare_parameter('min_v', -0.3)  #-1.0
      self.declare_parameter('max_v', 0.3)   #1.0
      self.declare_parameter('min_w', -1.5)  #-1.5
      self.declare_parameter('max_w', 1.5)   #1.5
        
      #retrieve parameter values
      self.rate = self.get_parameter('rate').value

      self.path_points = []

      
   def _init_state_variables(self):
      #initialise state variables
      self.current_state = None

      #initialise previous control
      self.previous_control = np.zeros(2)
        
      #initialise data storage for robot trajectory
      self.actual_trajectory = []
        
   def _init_communication(self):
      #initialise ROS publishers and subscribers
      self.cmd_vel_pub = self.create_publisher(Twist, '/a200_0656/twist_marker_server/cmd_vel', 10)
      self.odom_sub = self.create_subscription(Odometry, '/camera_odom', self.odom_callback, 10)
      self.navigan_sub = self.create_subscription(Path, '/navigan_path', self.navigan_callback, 10)

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
      
      #timer for control loop
      self.timer = self.create_timer(1.0/self.rate, self.control_loop)

      #DELETE
      #path-following behaviour options: 'stop' or 'repeat'
      self.path_type = self.PATH_TYPE
      self.stop = False 
   
   def odom_callback(self, msg):
   #function to process odometry messages
      #extract current position
      self.x_position = msg.pose.pose.position.x
      self.y_position = msg.pose.pose.position.y
        
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

   def navigan_callback(self, msg):
   #function to process navigan path messages
      #clear old points
      self.path_points = []
      #extract path points
      for pose in msg.poses:
         x = pose.pose.position.x
         y = pose.pose.position.y
         orientation_q = pose.pose.orientation
         _, _, th = self.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
         self.path_points.append([x, y, th])

      self.get_logger().info(f"Navigan callback recieved with {len(self.path_points)} points.")

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
   
   def send_data(self):
   #function to send trajectory data to external plotter
      trajectory_data ={
         'actual_x' : float(self.current_state[0]),
         'actual_y' : float(self.current_state[1]),
         'forecast_x': self.controller.next_states[:, 0].tolist() if hasattr(self.controller, 'next_states') else [],
         'forecast_y': self.controller.next_states[:, 1].tolist() if hasattr(self.controller, 'next_states') else [], 
         'navigan_x' : [pt[0] for pt in self.path_points],
         'navigan_y' : [pt[1] for pt in self.path_points],
      }

      #send data as json encoded udp
      self.socket.sendto(json.dumps(trajectory_data).encode(), self.PLOTTER_ADDRESS)

   def unwrap_current_state(self, current_state, ref_trajectory):
   #unwrap current yaw relative to reference trajectory
      ref_angle = ref_trajectory[0][2]
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

      if not self.path_points:
         #wait for path points
         self.get_logger().info('Waiting for path points...')
         return
        
      else:
         #get reference trajectory for next N steps
         ref_trajectory = self.path_points

         #create array of reference controls based on previous controls (ie minimise change in control)
         ref_controls = np.tile(self.previous_control, (self.controller.N, 1))

         #unwrap current state
         current_state_unwrapped = self.unwrap_current_state(self.current_state, ref_trajectory)

         #solve nmpc problem
         try:
            self.optimal_control = self.controller.solve(current_state_unwrapped, ref_trajectory, ref_controls)
         except Exception as e:
            self.get_logger().error(f"NMPC solver failed: {e}")
            return
        
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
   navigan_nmpc_node = NaviganNMPCNode()
   try:
      rclpy.spin(navigan_nmpc_node)
   except KeyboardInterrupt:
      pass
   navigan_nmpc_node.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import numpy as np
import socket
import json
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from .controller_class import Controller  
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy
)
from rclpy.duration import Duration

class NaviganNMPCNode(Node):
   PLOTTER_ADDRESS = ('192.168.131.61', 12345)

   def __init__(self):
      super().__init__('navigan_nmpc_controller_node')   
        
      self._init_parameters()
      self._init_state_variables()
      self._init_communication()
      self._init_controller()
      

   def _init_parameters(self):
      #load parameters
      self.get_logger().info("Loading parameters...")

      self.declare_parameter('rate', 20)
      self.declare_parameter('min_v', -0.5)  #-1.0
      self.declare_parameter('max_v', 0.5)   #1.0
      self.declare_parameter('min_w', -0.4)  #-1.5
      self.declare_parameter('max_w', 0.4)   #1.5
      self.declare_parameter('N', 10)
        
      #retrieve parameter values
      self.rate = self.get_parameter('rate').value

      self.path_points = []
      self.navigan_x = []
      self.navigan_y = []

      
   def _init_state_variables(self):
      #initialise goal position
      self.goal_position = None

      #initialise state variables
      self.current_state = None

      #initialise previous control
      self.previous_control = np.zeros(2)
        
      #initialise data storage for robot trajectory
      self.actual_trajectory = []
        
   def _init_communication(self):
      #initialise ROS publishers and subscribers
      sensor_qos, reliable_qos, fast_qos, parameter_qos = self.get_common_qos_profiles()
      qos_profile = QoSProfile(depth=10)
      # qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
      
      self.cmd_vel_pub = self.create_publisher(Twist, '/a200_0656/twist_marker_server/cmd_vel', 10)
      self.odom_sub = self.create_subscription(Odometry,'/zed/zed_node/odom' , self.odom_callback, 10) #'/vtr/odometry'
      self.navigan_sub = self.create_subscription(Path, '/vtr/planning_path', self.navigan_callback, qos_profile=qos_profile) #fast_qos)
      self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, qos_profile=reliable_qos)

      #wait for initial position
      self.initial_position_received = False
      self.initial_position = None
      while not self.initial_position_received:
         rclpy.spin_once(self)
    
      #create UDP socket for sending data to external plotter#create udp socket to send trajectory data to external plotter
      self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
   def _init_controller(self):
      #get controller parameters
      self.get_logger().info("starting controller")
      min_v = self.get_parameter('min_v').value
      max_v = self.get_parameter('max_v').value
      min_w = self.get_parameter('min_w').value
      max_w = self.get_parameter('max_w').value
      N = self.get_parameter('N').value
        
      #initialise nmpc controller with initial position and parameters
      self.controller = Controller(min_v, max_v, min_w, max_w, N, T=1.0/self.rate)
      
      #timer for control loop
      self.timer = self.create_timer(1.0/self.rate, self.control_loop)
   
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

   def goal_pose_callback(self, msg):
   #function to process goal pose messages
      self.goal_position = msg.pose
      self.get_logger().info(f"Goal position updated: {self.goal_position.position.x}, {self.goal_position.position.y}")
    
   def navigan_callback(self, msg):
      rclpy.logging.get_logger('navigan_nmpc_controller_node').info("Received Navigan path message.")
   #function to process navigan path messages
      #clear old points
      self.path_points = []
      self.navigan_x = []
      self.navigan_y = []
      #extract path points
      for pose in msg.poses:
         x = pose.pose.position.x -self.initial_position[0]
         y = pose.pose.position.y -self.initial_position[1]
         orientation_q = pose.pose.orientation
         _, _, th = self.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
         th = th - self.initial_position[2]  
         self.path_points.append([x, y, th])
         self.navigan_x.append(x)
         self.navigan_y.append(y)

      #self.get_logger().info(f"Navigan callback recieved with {len(self.path_points)} points.")

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

   def send_data(self):
   #function to send trajectory data to external plotter
      trajectory_data ={
         'actual_x' : float(self.current_state[0]),
         'actual_y' : float(self.current_state[1]),
         'forecast_x': self.controller.next_states[:, 0].tolist() if hasattr(self.controller, 'next_states') else [],
         'forecast_y': self.controller.next_states[:, 1].tolist() if hasattr(self.controller, 'next_states') else [], 
         'navigan_x' : self.navigan_x,
         'navigan_y' : self.navigan_y,
         }

      #send data as json encoded udp
      self.socket.sendto(json.dumps(trajectory_data).encode(), self.PLOTTER_ADDRESS)
        
   def reachGoalCheck(self, goalPosition, _r=0.5):
   #check if close to goal position 
      #calculate the Euclidean distance between current position and goal
      dx = self.current_state[0] - goalPosition.position.x
      dy = self.current_state[1] - goalPosition.position.y
      distance = (dx**2 + dy**2) ** 0.5

      #check position tolerance
      if distance > _r:
         return False
      return True
      
   def control_loop(self):
   #control loop runs periodically
      if not self.odom_received:
         #wait for odometry data
         self.get_logger().info('No odometry data recieved. Waiting.')
         return

      if not self.path_points:
         #wait for path points
         self.get_logger().info('No navigan path recieved. Waiting.')
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
            success = self.controller.success
         except Exception as e:
            self.get_logger().error(f"NMPC solver failed: {e}")
            return
         
         if not success:
            self.get_logger().warn("NMPC solve failed. Skipping publish and forecast.")
            return

         cmd_vel_msg = Twist()

         # if self.goal_position is None:
         #    self.get_logger().warn("No goal recieved. Waiting.")
         #    return
         # else:
         #    if self.reachGoalCheck(self.goal_position, _r=0.5):
         #       self.get_logger().info("Goal reached. Stopping.")
         #       #stop robot if goal is reached
         #       cmd_vel_msg.linear.x = 0.0
         #       cmd_vel_msg.angular.z = 0.0
            # else:
               #apply optimal control inputs
         cmd_vel_msg.linear.x = float(self.optimal_control[0])
         cmd_vel_msg.angular.z = float(self.optimal_control[1])
         print(f"Publishing cmd_vel: linear.x={cmd_vel_msg.linear.x}, angular.z={cmd_vel_msg.angular.z}")

         self.cmd_vel_pub.publish(cmd_vel_msg)

         #send trajectory data for plotting
         self.send_data()

   def get_common_qos_profiles(self):
   # For sensor data (like camera feeds, laser scans)
      sensor_qos = QoSProfile(
         reliability=ReliabilityPolicy.BEST_EFFORT,
         durability=DurabilityPolicy.VOLATILE,
         history=HistoryPolicy.KEEP_LAST,
         depth=1
      )
      
      # For critical messages (like goals, system states)
      reliable_qos = QoSProfile(
         reliability=ReliabilityPolicy.RELIABLE,
         durability=DurabilityPolicy.TRANSIENT_LOCAL,
         history=HistoryPolicy.KEEP_LAST,
         depth=1
      )
      
      # For fast updates (like position updates)
      fast_qos = QoSProfile(
         reliability=ReliabilityPolicy.BEST_EFFORT,
         durability=DurabilityPolicy.VOLATILE,
         history=HistoryPolicy.KEEP_LAST,
         depth=1,
         deadline=Duration(seconds=0.1)
      )
      
      # For parameters and configurations
      parameter_qos = QoSProfile(
         reliability=ReliabilityPolicy.RELIABLE,
         durability=DurabilityPolicy.TRANSIENT_LOCAL,
         history=HistoryPolicy.KEEP_LAST,
         depth=1,
         lifespan=Duration(seconds=3600)  # 1 hour
      )
   
      return sensor_qos, reliable_qos, fast_qos, parameter_qos
   
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

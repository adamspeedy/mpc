#!/usr/bin/env python3
import numpy as np
import csv
import socket
import json
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from people_msgs.msg import People
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

class SwitchingNMPCNode(Node):
   PLOTTER_ADDRESS = ('196.24.165.132', 12345)

   def __init__(self):
      super().__init__('switching_nmpc_controller_node')   
        
      self._init_parameters()
      self._init_state_variables()
      self._init_communication()
      self._init_controller()

   def _init_parameters(self):
   #load parameters
      self.get_logger().info("Loading parameters...")

      self.declare_parameter('rate', 20)
      self.declare_parameter('trajectory_file', '/home/administrator/code/nmpc_ws/data/trajectories/recorded_odometry.csv')
      self.declare_parameter('min_v', -1.0)  #-1.0
      self.declare_parameter('max_v', 1.0)   #1.0
      self.declare_parameter('min_w', -1.5)  #-1.5
      self.declare_parameter('max_w', 1.5)   #1.5
      self.declare_parameter('N', 5)
      self.declare_parameter('controller_switch_delay', 1.0)
      
      self.rate = self.get_parameter('rate').value
      self.N = self.get_parameter('N').value
      self.controller_switch_delay = self.get_parameter('controller_switch_delay').value

      #initialise reference trajectory file
      self.csv_file = self.get_parameter('trajectory_file').value

   def _init_state_variables(self):
   #initialise state variables
      #initialise current state
      self.current_state = None
      self.odom_received = False

      #initialise previous control
      self.previous_control = np.zeros(2)
        
      #initialise data storage for robot trajectory
      self.actual_trajectory = []
        
      # Load reference trajectory from csv file
      try:
         self.reference_trajectory = self.load_trajectory()
         self.get_logger().info(f"Loaded {len(self.reference_trajectory)} reference trajectory points")
      except Exception as e:
         self.get_logger().error(f"Failed to load trajectory: {e}")
         self.reference_trajectory = np.array([[0.0, 0.0, 0.0]])

      #initialise navigan trajectory
      self.path_points = []
      self.navigan_x = []
      self.navigan_y = []

      #initialise people detected
      self.people_detected = False
      self.people_positions = []

      #intitialise controller type
      self.active_behaviour = 'path_following'
      
      #
      self.stop = False
      self.path_type = 'repeat'

   def _init_communication(self):
      #initialise qos profiles
      self.sensor_qos, self.reliable_qos, self.fast_qos, self.parameter_qos = self.get_common_qos_profiles()

      #initialise ROS publishers and subscribers
      self.cmd_vel_pub = self.create_publisher(Twist, '/a200_0656/twist_marker_server/cmd_vel', 10)
      self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', self.reliable_qos)

      self.odom_sub = self.create_subscription(Odometry, '/camera_odom', self.odom_callback, 10)
      self.navigan_sub = self.create_subscription(Path, '/navigan_path', self.navigan_callback, self.fast_qos)
      self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, self.reliable_qos)
      self.people_sub = self.create_subscription(People, '/people', self.people_callback, self.sensor_qos)

      #wait for initial position
      self.initial_position_received = False
      self.initial_position = None
      while not self.initial_position_received:
         rclpy.spin_once(self)
    
      #create udp socket to send trajectory data to external plotter
      self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

      #send dummy goal to activate navigan
      self.publish_dummy_goal()
    
   def _init_controller(self):
   #get controller parameters
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
      
   def navigan_callback(self, msg):
   #function to process navigan path messages
      #clear old points
      self.path_points = []
      self.navigan_x = []
      self.navigan_y = []

      if len(msg.poses) == 0:
         self.get_logger().warning("Received empty path from Navigan")
         return

      #extract path points
      for pose in msg.poses:
         x = pose.pose.position.x
         y = pose.pose.position.y
         orientation_q = pose.pose.orientation
         _, _, th = self.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
         self.path_points.append([x, y, th])
         self.navigan_x.append(x)
         self.navigan_y.append(y)

   def people_callback(self, msg):
   #function to process people messages
      
      self.people_positions = [(person.position.x, person.position.y) for person in msg.people]

      #check if people are detected
      if not self.people_positions:
         if self.people_detected:
            self.get_logger().info("No people detected.")
         self.people_detected = False
      else:
         if not self.people_detected:
            self.get_logger().info("People detected in the scene.")
         self.people_detected = True

   def goal_pose_callback(self, msg):
   #function to process goal pose messages
      self.goal_position = msg.pose
      self.get_logger().info(f"Goal position updated: {self.goal_position}")

   def publish_dummy_goal(self):
      dummy_goal = PoseStamped()
      dummy_goal.header.frame_id = 'map'
      dummy_goal.pose.position.x = self.reference_trajectory[self.N][0]
      dummy_goal.pose.position.y = self.reference_trajectory[self.N][1]
      dummy_goal.pose.orientation.w = 1.0

      self.get_logger().info("Publishing dummy goal to trigger NaviGAN.")
      self.goal_pub.publish(dummy_goal)
   
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

      #find index of closest point
      closest_index = np.argmin(cost)
    
      #check if close to end of trajectory
      if closest_index >= len(self.reference_trajectory) - self.controller.N:
         #if end of trajectory is reached and stop is requested
         if self.path_type == 'stop':
            print('Stop')
            self.stop = True

         #if close to end of trajectory and repeat is requested
         elif self.path_type == 'repeat':
            print('Return to start')
            return 1
      
      return closest_index

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
        
      return ref_traj
   
   def check_behaviour(self):
      past_behaviour = self.active_behaviour

      if self.people_detected:
         #switch to navigan controller
         self.active_behaviour = 'navigan'
      else:
         #switch to nmpc controller
         self.active_behaviour = 'path_following'

      #check if controller has changed
      if self.active_behaviour != past_behaviour:
         self.get_logger().info(f"Switching controller from {past_behaviour} to {self.active_behaviour}")
         
         #stop the robot
         self.stop = True
         
         #set goal if switching to navigan
         if self.active_behaviour == 'navigan_nmpc':
            self.set_goal_position()

   def get_controller_trajectory(self):
   #function to get reference trajectory for path following or navigan behaviour
      ref_trajectory = []

      if self.active_behaviour == 'path_following':
         self.stop = False
         ref_trajectory = self.reference_trajectory_N()

      elif self.active_behaviour == 'navigan':
         self.stop = False
         if len(self.path_points) > 0:
            ref_trajectory = np.array(self.path_points)
         else:
            self.get_logger().warning("No navigan path points available. Stopping.")
            self.stop = True

      return ref_trajectory
   
   def set_goal_position(self):
   #function to set goal position
      traj = self.reference_trajectory_N()

      #get point to send as goal from reference trajectory
      goal_point = traj[self.N]

      goal_msg = PoseStamped()
      goal_msg.header.frame_id = 'map'
      goal_msg.pose.position.x = goal_point[0]
      goal_msg.pose.position.y = goal_point[1]
      yaw = goal_point[2]
      goal_msg.pose.orientation.z = np.sin(yaw/2)
      goal_msg.pose.orientation.w = np.cos(yaw/2)

      self.goal_pub.publish(goal_msg)

   def send_data(self):
   #function to send trajectory data to external plotter

      #send empty navigan if path following
      if self.active_behaviour == 'path_following':
         self.navigan_x = []
         self.navigan_y = []

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

   def control_loop(self):
   #control loop runs periodically
      if not self.odom_received:
         #wait for odometry data
         self.get_logger().info('Waiting for initial odometry data...')
         return
        
      else:
         #check which control behavious is needed
         self.check_behaviour()

         #get reference trajectory for desired behaviour
         ref_trajectory = self.get_controller_trajectory()

         #create array of reference controls based on previous controls (ie minimise change in control)
         ref_controls = np.tile(self.previous_control, (self.controller.N, 1))

         #unwrap current state
         current_state_unwrapped = self.unwrap_current_state(self.current_state, ref_trajectory)

         #solve nmpc problem
         self.optimal_control = self.controller.solve(current_state_unwrapped, ref_trajectory, ref_controls)
        
         #create and publish velocity command
         cmd_vel_msg = Twist()
         if self.stop == True:
            #stop
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            self.get_logger().debug("Robot stopped")
         else:
            #apply optimal control inputs
            cmd_vel_msg.linear.x = float(self.optimal_control[0])
            cmd_vel_msg.angular.z = float(self.optimal_control[1])
    
         self.cmd_vel_pub.publish(cmd_vel_msg)

         self.get_logger().debug(f"Following trajectory: {self.active_behaviour}")

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
      self.get_logger().info("Shutting down NMPC controller node")
      try:
         self.socket.close()
      except Exception as e:
         self.get_logger().error(f"Error closing socket: {e}")
      super().destroy_node()

def main(args=None):
   rclpy.init(args=args)
   try:
      switching_nmpc_node = SwitchingNMPCNode()
      try:
         rclpy.spin(switching_nmpc_node)
      except KeyboardInterrupt:
         switching_nmpc_node.get_logger().info("Keyboard interrupt received")
      except Exception as e:
         switching_nmpc_node.get_logger().error(f"Error during node execution: {e}")
      finally:
         switching_nmpc_node.destroy_node()
   except Exception as e:
      print(f"Error initializing node: {e}")
   finally:
      rclpy.shutdown()


if __name__ == '__main__':
    main()

    
   
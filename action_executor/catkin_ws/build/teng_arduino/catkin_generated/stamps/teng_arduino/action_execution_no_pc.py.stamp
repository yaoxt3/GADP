#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from franky import *
import franky
import threading
from std_msgs.msg import Int16, Float32, Bool
from teng_arduino.msg import TwistGripper, ObsCache
import pyrealsense2 as rs
import numpy as np
import pickle
import copy
import traceback
import os
from scipy.spatial.transform import Rotation

def quaternion_to_euler(q):
    
    q_x, q_y, q_z, q_w = q[0], q[1], q[2], q[3]
    roll = np.arctan2(2.0 * (q_w * q_x + q_y * q_z), 1.0 - 2.0 * (q_x**2 + q_y**2))
    pitch = np.arcsin(2.0 * (q_w * q_y - q_z * q_x))
    yaw = np.arctan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y**2 + q_z**2))

    return roll, pitch, yaw

class FrankaRobotController:
    def __init__(self):
        rospy.init_node('franka_robot_controller')
        robot_ip = rospy.get_param('~robot_ip')
        self.robot = Robot(robot_ip)
        self.robot.recover_from_errors()
        self.robot.relative_dynamics_factor = 0.1
        #self.robot.velocity_rel = 0.4
        #self.robot.acceleration_rel = 0.2
        #self.robot.jerk_rel = 0.02
        self.lock = threading.Lock()
        self.gripper = franky.Gripper(robot_ip)
        self.gripper_speed = 0.02
        print('Connecting to robot at {}'.format(robot_ip))
        self.get_initial_gripper_command = False
        target_positions = [0.2, 0, -0.28, -2.63, -0.03, 2.61, 0.75]
        joint_state_target = JointState(position=target_positions)
        waypoint = JointWaypoint(target=joint_state_target, reference_type=ReferenceType.Absolute)
        init_motion = JointWaypointMotion([waypoint])
        self.robot.move(init_motion)
        
        self.gripper_sub = rospy.Subscriber('/franka_gripper_command', Int16, self.gripper_open_callback, queue_size=10)
        self.twist_gripper_sub = rospy.Subscriber('/franka_twist_gripper', TwistGripper, self.twist_gripper_callback, queue_size=10)
        self.obs_pub = rospy.Publisher('/observation_data', ObsCache, queue_size=1)
        self.action_done_pub = rospy.Publisher('action_done', Bool, queue_size=1)
        
        
        
        self.translation = [0, 0, 0]
        self.angular_y = 0.0
        self.movement = [0, 0, 0, 0]
        self.new_command_received = False
        self.new_gripper_command_received = False
        self.gripper_cnt = 0
        self.gripper_command = 0
        self.max_gripper_width = 0.08
        self.gripper_open = 0
        self.gripper_force = 5
        
        self.data_cache = {
            'agent_pose': []
        }
        
        
        self.motion_done_event = threading.Event()
        
        self.control_thread = threading.Thread(target=self.robot_control_loop)
        self.control_thread.start()
        
        self.obs_thread = threading.Thread(target=self.obs_loop)
        self.obs_thread.start()
        
        self.gripper_thread = threading.Thread(target=self.gripper_control_loop)
        self.gripper_thread.start()
        

    def twist_gripper_callback(self, msg):
        with self.lock:
            self.movement = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, msg.twist.angular.y]
            self.movement[1] = (-1) * self.movement[1]
            self.movement[2] = (-1) * self.movement[2]
            # print("x, y, z action:", self.movement[:3])
            self.gripper_command = msg.gripper
            self.new_command_received = True
            print("receive motion:", self.movement)
    
    def gripper_open_callback(self, msg):
        with self.lock:
            if msg.data == 0 and self.gripper_open == False:
                self.gripper_open = True
                self.new_gripper_command_received = True
            elif msg.data == 1 and self.gripper_open == True:
                self.gripper_open = False
                self.new_gripper_command_received = True
                self.get_initial_gripper_command = True
    
    def robot_control_loop(self):
        while not rospy.is_shutdown():
            if any(self.movement) and self.new_command_received:
                translation = np.array([self.movement[0], self.movement[1], self.movement[2]])
                quat = Rotation.from_euler("xyz", [0, 0, 0]).as_quat()
                
                way = Affine(translation, quat)
                motion = CartesianMotion(way, ReferenceType.Relative)
                print("start move:", self.movement)  # This should now print correctly
                self.robot.move(motion)
                rospy.loginfo("Motion completed successfully.")
                self.motion_done_event.set()  # Signal motion completion

                with self.lock:
                    self.new_command_received = False  # Reset only after motion execution

                
    def obs_loop(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
                cartesian_state = self.robot.current_cartesian_state
                pose = cartesian_state.pose
                self.gripper_width = self.gripper.width
                # for 0.14 gripper
                if self.get_initial_gripper_command:
                    self.gripper_width = 0.08 - (0.08 - self.gripper_width) * 67 / 80
                # for 0.08 gripper
                # if self.get_initial_gripper_command:
                #     self.gripper_width = max(0, 0.08 - (0.08 - self.gripper_width) * 67 / 52)
                # for 0.0625 gripper
                # if self.get_initial_gripper_command:
                #     self.gripper_width = max(0, 0.08 - (0.08 - self.gripper_width) * 67 / 43)
                # for 0.08 gripper
                # if self.get_initial_gripper_command:
                #     self.gripper_width = max(0, 0.08 - (0.08 - self.gripper_width) * 67 / 72)
                print("gripper width:", self.gripper_width)
                x, y, z = pose.end_effector_pose.translation.flatten()
                q = pose.end_effector_pose.quaternion.flatten()
                roll, pitch, yaw = quaternion_to_euler(q)
                pose_arr = np.array([x, y, z, roll, pitch, yaw, self.gripper_width], dtype=np.float32)
                
                with self.lock:
                    self.data_cache['agent_pose'].append(copy.deepcopy(pose_arr))
                    
                    if len(self.data_cache['agent_pose']) >= 2:
                        latest_pose_1 = self.data_cache['agent_pose'][-2]
                        latest_pose_2 = self.data_cache['agent_pose'][-1]
                
                        obs_cache = ObsCache()
                        obs_cache.pose_1 = latest_pose_1.flatten().tolist()
                        obs_cache.pose_2 = latest_pose_2.flatten().tolist()
                        
                        self.obs_pub.publish(obs_cache)
                    else:
                        latest_pose_1 = self.data_cache['agent_pose'][-1]
                        latest_pose_2 = latest_pose_1
                
                        obs_cache = ObsCache()
                        obs_cache.pose_1 = latest_pose_1.flatten().tolist()
                        obs_cache.pose_2 = latest_pose_2.flatten().tolist()
                        
                        self.obs_pub.publish(obs_cache)
                
                rate.sleep() 
    
    def gripper_control_loop(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            with self.lock:
                if self.new_gripper_command_received:
                    current_gripper_move = self.gripper_open

            if self.new_gripper_command_received:
                if current_gripper_move:
                    current_future = self.gripper.open_async(self.gripper_speed)
                elif not current_gripper_move:
                    current_future = self.gripper.grasp_async(0.03, self.gripper_speed, self.gripper_force, epsilon_outer=0.2)
            
                with self.lock:
                    self.new_gripper_command_received = False
                    
            rate.sleep()
     
    def run(self):
        self.gripper.move_async(self.gripper.max_width/2, self.gripper_speed)
        print('Gripper opened: {}'.format(self.gripper.max_width))
        rate = rospy.Rate(5) 
        while not rospy.is_shutdown():
            if self.motion_done_event.is_set():
                self.action_done_pub.publish(Bool(data=True))  # Publish True when motion is done
                rospy.loginfo("Published True for action_done.")
                self.motion_done_event.clear()  # Reset the event after publishing
            else:
                self.action_done_pub.publish(Bool(data=False))  # Publish False otherwise
            rate.sleep()
            
if __name__ == '__main__':
    controller = FrankaRobotController()
    controller.run()

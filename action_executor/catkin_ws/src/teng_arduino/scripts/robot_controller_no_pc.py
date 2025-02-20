#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from franky import *
import franky
import threading
from std_msgs.msg import Int16, Float32MultiArray
from teng_arduino.msg import TwistGripper, PoseAction
import pyrealsense2 as rs
import numpy as np
import pickle
import copy
import traceback
import os
from scipy.spatial.transform import Rotation
from pynput import keyboard
import random
    
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
        self.robot.relative_dynamics_factor = 0.05
        self.robot.velocity_rel = 0.4
        self.robot.acceleration_rel = 0.2
        self.robot.jerk_rel = 0.02
        self.gripper = franky.Gripper(robot_ip)
        self.gripper_speed = 0.02
        self.gripper_force = 5
        print('Connecting to robot at {}'.format(robot_ip))
        
        # target_positions = [0.17608869144791048, 0.14026744660612564, -0.14517009432783834, -2.1095372149925256, -0.06725750007232029, 2.289596436017825, 0.9523127157392397]
        # joint_state_target = JointState(position=target_positions)
        # waypoint = JointWaypoint(target=joint_state_target, reference_type=ReferenceType.Absolute)
        # init_motion = JointWaypointMotion([waypoint])
        # self.robot.move(init_motion)
        
        # target_positions = [0.2, 0, -0.28, -2.63, -0.03, 2.61, 0.75]
        # joint_state_target = JointState(position=target_positions)
        # waypoint = JointWaypoint(target=joint_state_target, reference_type=ReferenceType.Absolute)
        # init_motion = JointWaypointMotion([waypoint])
        # self.robot.move(init_motion)
        
        self.gripper_sub = rospy.Subscriber('/franka_gripper_command', Int16, self.gripper_open_callback, queue_size=1)
        self.twist_gripper_sub = rospy.Subscriber('/franka_twist_gripper', TwistGripper, self.twist_gripper_callback, queue_size=1)
        self.pose_action_pub = rospy.Publisher('/franka_pose_action', PoseAction, queue_size=1)
        self.stop_pub =  rospy.Publisher('stop_command', Int16, queue_size=1)
        
        self.translation = [0, 0, 0]
        self.angular_y = 0.0
        self.movement = [0, 0, 0, 0]
        self.new_command_received = False
        self.new_grippr_command_received = False
        self.gripper_cnt = 0
        self.gripper_command = 0
        self.max_gripper_width = 0.08
        
        self.gripper_open = False
        
        self.lock = threading.Lock()
        
        self.control_thread = threading.Thread(target=self.robot_control_loop)
        self.control_thread.start()
        
        self.record_thread = threading.Thread(target=self.robot_record_loop)
        self.record_thread.start()
        
        self.gripper_thread = threading.Thread(target=self.gripper_control_loop)
        self.gripper_thread.start()
        
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()


    def on_key_press(self, key):
        # Callback for keyboard event
        try:
            if key.char == 's':
                rate = rospy.Rate(10)
                for i in range(10):
                    self.stop_pub.publish(Int16(data=0))
                    rate.sleep()
                rospy.signal_shutdown("shutting down")
        except AttributeError:
            pass
        
    def twist_gripper_callback(self, msg):
        with self.lock:
            self.movement = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, msg.twist.angular.y]
            self.gripper_command = msg.gripper
            self.new_command_received = True
    
    def gripper_open_callback(self, msg):
        with self.lock:
            if msg.data == 1 and self.gripper_open == False:
                self.gripper_open = True
                self.new_grippr_command_received = True
            elif msg.data == 2 and self.gripper_open == True:
                self.gripper_open = False
                self.new_grippr_command_received = True
        
    def robot_control_loop(self):
        rate = rospy.Rate(2) 
        while not rospy.is_shutdown():
            # with self.lock:
            #     if self.new_command_received:
            #         movement = self.movement
            #         self.new_command_received = False
            #     else:
            #         movement = [0, 0, 0, 0]
                            
            if any(self.movement):
                translation = np.array([self.movement[0], self.movement[1], self.movement[2]])
                quat = Rotation.from_euler("xyz", [0, self.movement[3], 0]).as_quat()
                way = Affine(translation, quat)
                motion_forward = CartesianMotion(way, ReferenceType.Relative)
                self.robot.move(motion_forward, asynchronous=True)
                
            rate.sleep()
            
    def robot_record_loop(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            cartesian_state = self.robot.current_cartesian_state
            pose = cartesian_state.pose
            self.gripper_width = self.gripper.width

            x, y, z = pose.end_effector_pose.translation.flatten()
            q = pose.end_effector_pose.quaternion.flatten()
            roll, pitch, yaw = quaternion_to_euler(q)
            pose_arr = np.array([x, y, z, roll, pitch, yaw, self.gripper_width], dtype=np.float32)
            
            m1 = (-1) * self.movement[1]
            m2 = (-1) * self.movement[2]
            action = np.array([self.movement[0], m1, m2, 0, self.movement[3], 0, self.gripper_open], dtype=np.float32) 
            print(pose_arr)
            pose_action_msg = PoseAction()
            pose_action_msg.pose_data = Float32MultiArray(data=pose_arr)
            pose_action_msg.action_data = Float32MultiArray(data=action) 
            
            self.pose_action_pub.publish(pose_action_msg)   
            self.stop_pub.publish(Int16(data=1))   
            
            rate.sleep()
            
    def gripper_control_loop(self):
        rate = rospy.Rate(2)
        #current_future = None
        while not rospy.is_shutdown():
            with self.lock:
                if self.new_grippr_command_received:
                    current_gripper_move = self.gripper_open

            if self.new_grippr_command_received:
                if current_gripper_move:
                    current_future = self.gripper.open_async(self.gripper_speed)
                elif not current_gripper_move:
                    current_future = self.gripper.grasp_async(0.03, self.gripper_speed, self.gripper_force, epsilon_outer=0.2)
                    
            with self.lock:
                self.new_grippr_command_received = False
            rate.sleep()

    def run(self):
        self.gripper.move_async(self.gripper.max_width/2, self.gripper_speed)
        print('Gripper opened: {}'.format(self.gripper.max_width/2))
        rospy.spin()

if __name__ == '__main__':
    controller = FrankaRobotController()
    controller.run()
    
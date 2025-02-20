#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from franky import *
import franky
import threading
from std_msgs.msg import Int16, Float32, Bool
from teng_arduino.msg import TwistGripper
import pyrealsense2 as rs
import numpy as np
import pickle
import copy
import traceback
import os
from scipy.spatial.transform import Rotation
import open3d as o3d
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg

    
def quaternion_to_euler(q):
    
    q_x, q_y, q_z, q_w = q[0], q[1], q[2], q[3]
    roll = np.arctan2(2.0 * (q_w * q_x + q_y * q_z), 1.0 - 2.0 * (q_x**2 + q_y**2))
    pitch = np.arcsin(2.0 * (q_w * q_y - q_z * q_x))
    yaw = np.arctan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y**2 + q_z**2))

    return roll, pitch, yaw

class FrankaRobotController:
    def __init__(self):
        self.lock = threading.Lock()
        rospy.init_node('franka_robot_controller')
        robot_ip = rospy.get_param('~robot_ip')
        self.robot = Robot(robot_ip)
        self.robot.recover_from_errors()
        self.robot.relative_dynamics_factor = 0.05
        self.robot.velocity_rel = 0.4
        self.robot.acceleration_rel = 0.2
        self.robot.jerk_rel = 0.02
        print('Connecting to robot at {}'.format(robot_ip))
        
        # # target_positions = [0.2, 0, -0.28, -2.63, -0.03, 2.61, 1.58]
        # target_positions = [0.2, 0, -0.28, -2.63, -0.03, 2.61, 0.75]
        # joint_state_target = JointState(position=target_positions)
        # waypoint = JointWaypoint(target=joint_state_target, reference_type=ReferenceType.Absolute)
        # init_motion = JointWaypointMotion([waypoint])
        self.action_done_pub = rospy.Publisher('action_done', Bool, queue_size=1)   
        # self.robot.move(init_motion)
        self.action_done_pub.publish(True)

        self.new_command_received = False
        self.new_grippr_command_received = False
        self.get_initial_gripper_command = False
        self.movement = [0, 0, 0, 0]
        self.new_gripper_command_received = False
        self.gripper_open = False
        self.gripper_width = 0.039
        self.gripper_pub = rospy.Publisher("/Robotiq2FGripperRobotOutput", outputMsg.Robotiq2FGripper_robot_output, queue_size=10)
        self.gripper_info_sub = rospy.Subscriber("/Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, self.gripper_info_callback, queue_size=1)
        self.gripper_sub = rospy.Subscriber('/franka_gripper_command', Int16, self.gripper_open_callback, queue_size=1)
        self.twist_gripper_sub = rospy.Subscriber('/franka_twist_gripper', TwistGripper, self.twist_gripper_callback, queue_size=1)  
        
        self.command = self.gripper_init() 
        self.reverse = False
        
        self.data_cache = {
            'point_cloud': [],
            'agent_pose': [],
            'action': []
        }
        
        self.motion_done_event = threading.Event()
        
        self.gripper_thread = threading.Thread(target=self.gripper_control_loop)
        self.gripper_thread.start()
        
        self.control_thread = threading.Thread(target=self.robot_control_loop)
        self.control_thread.start()
        
        self.record_thread = threading.Thread(target=self.robot_record_loop)
        self.record_thread.start()
        
        
    def twist_gripper_callback(self, msg):
        with self.lock:
            self.movement = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, msg.twist.angular.y]
            self.new_command_received = True
    
    def gripper_open_callback(self, msg):
        with self.lock:
            if msg.data == 1 and self.gripper_open == False:
                self.gripper_open = True
                self.new_gripper_command_received = True
                self.get_initial_gripper_command =True
                print("open gripper")
            elif msg.data == 2 and self.gripper_open == True:
                self.gripper_open = False
                self.new_gripper_command_received = True
                self.get_initial_gripper_command = True
                print("close gripper")
    
    def save_data(self, directory = '/home/tumi6/yxt/pkl_data/10-31/panda_pick_455_1'):    
        cnt = len(os.listdir(directory))
        path = os.path.join(directory, 'traj'+str(cnt)+'.pkl')
        print(path)
        with open(path, 'wb') as f:
            pickle.dump(self.data_cache, f)
        
    def robot_control_loop(self):           
        rate = rospy.Rate(2)  # Publish at 2 Hz
        while not rospy.is_shutdown():

            if any(self.movement):
                try:
                    # Start the motion asynchronously
                    translation = np.array([self.movement[0], self.movement[1], self.movement[2]])
                    quat = Rotation.from_euler("xyz", [0, 0, self.movement[3]]).as_quat()
                    way = Affine(translation, quat)
                    motion = CartesianMotion(way, ReferenceType.Relative)
                    self.robot.move(motion, asynchronous=True)

                    # Wait for motion to complete
                    self.robot.join_motion()
                    rospy.loginfo("Motion completed successfully.")

                    # Signal motion completion
                    self.motion_done_event.set()

                except Exception as e:
                    rospy.logerr(f"Motion failed: {e}")
                    self.motion_done_event.clear() 
            
            rate.sleep()
            
    def robot_record_loop(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            cartesian_state = self.robot.current_cartesian_state
            pose = cartesian_state.pose
            
            x, y, z = pose.end_effector_pose.translation.flatten()
            q = pose.end_effector_pose.quaternion.flatten()
            roll, pitch, yaw = quaternion_to_euler(q)
            pose_arr = np.array([x, y, z, roll, pitch, yaw, self.gripper_width], dtype=np.float32)
            print(pose_arr)
            rate.sleep()
    
    def gripper_info_callback(self, msg):
        if self.get_initial_gripper_command:
            self.gripper_width = max(0, 0.08 - (msg.gPO - 3) / 226 * (13/9) * 0.08)
        else:
            self.gripper_width = (-msg.gPO + 229) / 226 * 0.08
            
    def gripper_control_loop(self):
        """Main control loop for gripper control"""
        rate = rospy.Rate(10)
        # gripper init
        while not rospy.is_shutdown():
            if self.new_gripper_command_received:
                if self.gripper_open:
                    # Send gripper command
                    self.command = self.gripper_control_command(0, self.command)
                else:
                    self.command = self.gripper_control_command(255, self.command) 
                self.new_gripper_command_received = False
            rate.sleep()
    
    def gripper_activate(self):
        """ Activate the gripper. """
        command = outputMsg.Robotiq2FGripper_robot_output()
        command.rPR = 127
        command.rACT = 1
        command.rGTO = 1
        command.rSP = 255
        command.rFR = 5
        self.gripper_pub.publish(command)
        return command

    def gripper_reset(self):
        """ Reset the gripper to its inactive state. """
        command = outputMsg.Robotiq2FGripper_robot_output()
        command.rACT = 0
        self.gripper_pub.publish(command)
        print("reset")
        return command
    
    def gripper_init(self):
        count = 0
        while not rospy.is_shutdown():
            command = self.gripper_reset()
            rospy.sleep(0.05)
            count+=1
            if count > 10:
                break
        rospy.sleep(0.5)
        command = self.gripper_activate()
        rospy.sleep(0.5)
        print("gripper init")
        return command
    
    def gripper_control_command(self, state, command):
        # global gripper_pub
        position = state
        if position > 255:
            position = 255
        elif position < 0:
            position = 0      
        command.rPR = position #0-255 : 0-8.5cm
        self.gripper_pub.publish(command)
        return command

    def run(self):
        rospy.spin()
              
if __name__ == '__main__':
    controller = FrankaRobotController()
    controller.run()
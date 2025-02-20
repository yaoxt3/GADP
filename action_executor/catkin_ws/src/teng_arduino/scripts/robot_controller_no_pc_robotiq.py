#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from franky import *
import franky
import threading
from std_msgs.msg import Int16, Float32MultiArray
from teng_arduino.msg import TwistGripper, PoseAction
import numpy as np
from scipy.spatial.transform import Rotation
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg
from pynput import keyboard
    
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
        self.gripper_width = 0.04
                
        self.lock = threading.Lock()
        
        target_positions = [0.17608869144791048, 0.14026744660612564, -0.14517009432783834, -2.1095372149925256, -0.06725750007232029, 2.289596436017825, 0.9523127157392397]
        joint_state_target = JointState(position=target_positions)
        waypoint = JointWaypoint(target=joint_state_target, reference_type=ReferenceType.Absolute)
        init_motion = JointWaypointMotion([waypoint])
        self.robot.move(init_motion)
        
        target_positions = [0.2, 0, -0.28, -2.63, -0.03, 2.61, 0.75]
        joint_state_target = JointState(position=target_positions)
        waypoint = JointWaypoint(target=joint_state_target, reference_type=ReferenceType.Absolute)
        init_motion = JointWaypointMotion([waypoint])
        self.robot.move(init_motion)
        
        
        self.new_command_received = False
        self.new_gripper_command_received = False
        self.gripper_sub = rospy.Subscriber('/franka_gripper_command', Int16, self.gripper_open_callback, queue_size=1)
        self.twist_gripper_sub = rospy.Subscriber('/franka_twist_gripper', TwistGripper, self.twist_gripper_callback, queue_size=1)
        self.gripper_pub = rospy.Publisher("/Robotiq2FGripperRobotOutput", outputMsg.Robotiq2FGripper_robot_output, queue_size=10)
        self.gripper_info_sub = rospy.Subscriber("/Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, self.gripper_info_callback, queue_size=1) 
        self.pose_action_pub = rospy.Publisher('/franka_pose_action', PoseAction, queue_size=1)
        self.stop_pub =  rospy.Publisher('stop_command', Int16, queue_size=1)
        self.command = self.gripper_init()
        self.translation = [0, 0, 0]
        self.angular_y = 0.0
        self.movement = [0, 0, 0, 0]
        self.gripper_cnt = 0
        self.gripper_command = 0
        self.max_gripper_width = 0.08
        
        self.gripper_open = False
        
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
                self.new_gripper_command_received = True
            elif msg.data == 2 and self.gripper_open == True:
                self.gripper_open = False
                self.new_gripper_command_received = True
        
    def robot_control_loop(self):
        rate = rospy.Rate(2) 
        while not rospy.is_shutdown():
                            
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
            self.gripper_width = self.gripper_width

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
            
    def gripper_info_callback(self, msg):
        with self.lock:
            self.gripper_width = (-msg.gPO + 229) / 226 * 0.08
            
    def gripper_control_loop(self):
        """Main control loop for gripper control"""
        rate = rospy.Rate(2)
        # gripper init
        while not rospy.is_shutdown():
            if self.new_gripper_command_received:
                if self.gripper_open:
                    # Send gripper command
                    self.command = self.gripper_control_command(0, self.command)
                else:
                    self.command = self.gripper_control_command(255, self.command) 
                    
                with self.lock:
                    self.new_gripper_command_received = False
            rate.sleep()
    
    def gripper_activate(self):
        """ Activate the gripper. """
        command = outputMsg.Robotiq2FGripper_robot_output()
        command.rPR = 127 #255
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
    
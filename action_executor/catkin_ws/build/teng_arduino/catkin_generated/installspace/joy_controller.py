#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from frankx import Affine, LinearRelativeMotion, Robot, JointMotion
import threading
from std_msgs.msg import Int16, Float32
from teng_arduino.msg import TwistGripper
import math

class JoystickFrankaController:
    def __init__(self):
        rospy.init_node('joystick_franka_controller')

        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.gripper_pub = rospy.Publisher('/franka_gripper_command', Int16, queue_size=1)
        self.twist_gripper_pub = rospy.Publisher('/franka_twist_gripper', TwistGripper, queue_size=1)
        
        self.joy_control = Twist()
        self.xyz_move_diff = 0.02
        self.ryp_rot_diff = 0.1
        self.axis_x_index = 1
        self.axis_y_index = 0
        self.axis_z_index = 4
        self.button_xyz_increase_index = 0 # Buttion A
        self.button_xyz_decrease_index = 1 # Button B
        self.xyz_move_scale = 0.02

        
        # self.rot_x_index = 3
        # self.yaw_index = 4
        self.axis_rot_y_index0 = 7
        self.axis_rot_y_index1 = 2
        self.axis_rot_y_index2 = 5
        # self.rot_z_index = 6
        self.button_ryp_increase_index = 2 # Button X
        self.button_ryp_decrease_index = 3  # Button Y
        self.ryp_rot_scale = 0.01
        
        self.button_gripper_open_index = 4
        self.button_gripper_close_index = 5
        # self.gripper_index = 2
        self.dead_zone = 0.005
        self.max_gripper_width = 0.08
        

        
    def joy_callback(self, joy_msg):
        # twist = Twist()
        twist_gripper_msg = TwistGripper()
        twist_gripper_msg.twist.linear.x = joy_msg.axes[self.axis_x_index] * self.xyz_move_diff
        twist_gripper_msg.twist.linear.y = joy_msg.axes[self.axis_y_index] * self.xyz_move_diff
        twist_gripper_msg.twist.linear.z = joy_msg.axes[self.axis_z_index] * self.xyz_move_diff
        # angular_y1 = (1.0 - joy_msg.axes[self.axis_rot_y_index1]) 
        # angular_y2 = (1.0 - joy_msg.axes[self.axis_rot_y_index2])
        # twist_gripper_msg.twist.angular.y = (angular_y1 - angular_y2) * math.pi / 8
        
        twist_gripper_msg.twist.angular.y = joy_msg.axes[self.axis_rot_y_index0] * self.ryp_rot_diff * math.pi / 8
        
        # twist_gripper_msg.twist.angular.x = joy_msg.axes[self.roll_index] * self.ryp_rot_diff
        # twist_gripper_msg.twist.angular.y = joy_msg.axes[self.pitch_index] * self.ryp_rot_diff
        # twist_gripper_msg.twist.angular.z = joy_msg.axes[self.yaw_index] * self.ryp_rot_diff
        
        # twist_gripper_msg.gripper = (joy_msg.axes[self.gripper_index] + 1)*self.max_gripper_width/2 
        # gripper_width = (joy_msg.axes[self.gripper_index] + 1)*self.max_gripper_width/2 
        
        if joy_msg.buttons[self.button_xyz_increase_index] == 1:
            self.xyz_move_diff += self.xyz_move_scale
            print('Button 0 pressed')
        elif joy_msg.buttons[self.button_xyz_decrease_index] == 1:
            self.xyz_move_diff -= self.xyz_move_scale
            print('Button 1 pressed')
        elif joy_msg.buttons[self.button_ryp_increase_index] == 1:
            self.ryp_rot_diff += self.ryp_rot_scale
            print('Button 2 pressed')
        elif joy_msg.buttons[self.button_ryp_decrease_index] == 1:
            self.ryp_rot_diff -= self.ryp_rot_scale
            print('Button 3 pressed')
        
        self.xyz_move_diff = max(self.xyz_move_diff, 0.01)
        self.ryp_rot_diff = max(self.ryp_rot_diff, 0.01)
            
        # print('Move diff: {}'.format(self.xyz_move_diff), 'Rot diff: {}'.format(self.ryp_rot_diff))
        
        if all(abs(axis) < self.dead_zone for axis in [twist_gripper_msg.twist.linear.x, twist_gripper_msg.twist.linear.y, twist_gripper_msg.twist.linear.z]):
            twist_gripper_msg.twist.linear.x = 0
            twist_gripper_msg.twist.linear.y = 0
            twist_gripper_msg.twist.linear.z = 0
        
        if all(abs(axis) < self.dead_zone*2 for axis in [twist_gripper_msg.twist.angular.x, twist_gripper_msg.twist.angular.y, twist_gripper_msg.twist.angular.z]):
            twist_gripper_msg.twist.angular.x = 0
            twist_gripper_msg.twist.angular.y = 0
            twist_gripper_msg.twist.angular.z = 0
         
        # print('Publishing twist: {}'.format(twist_gripper_msg))
        self.twist_gripper_pub.publish(twist_gripper_msg)
        
        if joy_msg.buttons[self.button_gripper_open_index] == 1:
            #print('Gripper open button pressed')
            self.gripper_pub.publish(Int16(data=1))
        elif joy_msg.buttons[self.button_gripper_close_index] == 1:
            #print('Gripper close button pressed')
            self.gripper_pub.publish(Int16(data=2))
        else:
            self.gripper_pub.publish(Int16(data=3))
        
        
    def run(self):

        rospy.spin()
        
if __name__ == '__main__':
    controller = JoystickFrankaController()
    
    print('Joystick controller is running...')
    controller.run()
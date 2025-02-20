#!/usr/bin/env python

import numpy as np
import rospy
from teng_arduino.msg import channels
from threading import Lock
import time
import sys
import VADMI_Gripper as GP
from std_srvs.srv import SetBool



class PipeController:
    
    def __init__(self):
        self.flag = False
        self.dataLog = [] # data to be save as csv
        self.state = 0
        self.count = -1
        self.window_size = 10
        self.buffer_lock = Lock()  
        self.gripper = GP.Gripper("192.168.0.10")
        self.threshold1 = 3.3
        self.ref1 = np.empty(3)
        self.ref2 = np.empty(3)


        # state 0: wait for grasping
        # 0 --> 1: detect objects in grasping area
        # state 1: grasp & move 
        # 1 --> 2: finish moving
        # state 2: wait for picking up
        # 2 --> 3: detect human touching object
        # state 3: releasing gripper & move back to initial position
        
        self.sub_arduino = rospy.Subscriber("TENG_SIGNALS",channels,self.CB_signal)


    def get_mean_buffer(self):
        with self.buffer_lock:
            r = np.mean(self.buffer, axis = 0)
        return r
        

    def CB_signal(self, data):
        
        if self.count == -1:
            self.buffer = np.zeros([self.window_size, len(data.c)])

        self.count = (self.count + 1) % self.window_size
        with self.buffer_lock:
            self.buffer[self.count, :] = data.c
        
        if self.state == 0:
            m = np.mean(self.buffer, axis = 0)
            if np.max(m) > self.threshold1:
                self.state = 1
                print("state 0 --> 1")

        if self.state == 2:
            m = np.mean(self.buffer, axis = 0)
            jRate = (m - self.ref2) / (self.ref1 - self.ref2)
            jValue = m - self.ref2
            print("jValue: ", jValue, "ref2: ", self.ref2 )
            # np.max(jRate) > 0.6 or
            # if  np.max(jValue) > 2 or np.mean(m) > self.threshold1: 
            #     self.state = 3
            #     print("state 2 --> 3")


            if  np.max(jValue) > 0.9 or np.min(m) > 4.6: 
                self.state = 3
                print("state 2 --> 3")


    def call_move_A(self):
        rospy.wait_for_service('boolstartA')
        try:
            move_client_A = rospy.ServiceProxy('boolstartA', SetBool)
            resp1 = move_client_A(True)
            return resp1.success

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    
    def call_move_B(self):
        rospy.wait_for_service('boolstartB')
        try:
            move_client_B = rospy.ServiceProxy('boolstartB', SetBool)
            resp1 = move_client_B(True)
            return resp1.success

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)



def main(args):
    
    rospy.init_node('handle_node',anonymous=True)
    S = PipeController()

    while S.state == 0:
        pass

    if S.state == 1:

        time.sleep(0.05)        
        S.gripper.drop("on")
        time.sleep(0.2)
        S.ref1 = S.get_mean_buffer()

        time.sleep(0.5)
        # call client
        S.call_move_B()

        time.sleep(0.3)
        S.ref2 = S.get_mean_buffer()
        S.state = 2
        print("state 1 --> 2")

    while S.state == 2:
        pass

    if S.state == 3:
        time.sleep(0.1)
        S.gripper.drop("off")
        # call client & move back
        time.sleep(1)
        S.call_move_A()


    



    

if __name__ == '__main__':
    main(sys.argv)



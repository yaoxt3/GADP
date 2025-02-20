#!/usr/bin/env python

import numpy as np
import rospy
from teng_arduino.msg import channels
from std_msgs.msg import Int8
import sys
import time


################################
# created by yuvaram
#yuvaramsingh94@gmail.com
################################

class saver:
    
    def __init__(self):
        self.flag = False
        self.dataLog = [] # data to be save as csv
        self.timeLog = [] # time stamp
        
        self.sub_key = rospy.Subscriber("key",Int8,self.CB_key)
        self.sub_arduino = rospy.Subscriber("TENG_SIGNALS1",channels,self.CB_arduino)


    def CB_key(self, data):
        if data == 113:
            if self.flag == False:
                self.flag = True
                print("saving data")
                a = np.array([self.timeLog])
                b = np.array(self.dataLog)
                print(np.shape(a))
                print(np.shape(b))
                d_t = np.concatenate((a, np.transpose(b)), axis = 0)
                np.savetxt("fuu.csv", np.transpose(d_t), delimiter=",")
            



    def CB_arduino(self, data):
        self.timeLog.append(data.stamp.secs + data.stamp.nsecs * (1e-9))
        self.dataLog.append(data.c)
        # print("kkkk")




def main(args):
  
    rospy.init_node('saver',anonymous=True)
    S = saver()
    # time.sleep(20)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

  
if __name__ == '__main__':
    main(sys.argv)



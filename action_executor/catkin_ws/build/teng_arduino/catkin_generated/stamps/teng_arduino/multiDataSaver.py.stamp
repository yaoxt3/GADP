#!/usr/bin/env python

import numpy as np
import rospy
from teng_arduino.msg import channels
from std_msgs.msg import Int8
import sys


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
        self.sub_arduino = rospy.Subscriber("TENG_SIGNALS",channels,self.CB_arduino)


    def CB_key(self, data):
        print(data.data)
        if data.data == 113:
            if self.flag == False:
                self.flag = True
                print("saving data")
                a = np.array([self.timeLog])
                b = np.array(self.dataLog)
                print(np.shape(a))
                print(np.shape(b))
                d_t = np.concatenate((a, np.transpose(b)), axis = 0)
                np.savetxt("/home/yxt/catkin_ws/foo.csv", np.transpose(d_t), delimiter=",")
        



    def CB_arduino(self, data):
        self.timeLog.append(data.stamp.secs + data.stamp.nsecs * (1e-9))
        self.dataLog.append(data.c)
        # print(data.)




def main(args):
  
  rospy.init_node('saver',anonymous=True)
  S = saver()
  try:
    rospy.spin()
  except KeyboardInterrupt:
      pass
if __name__ == '__main__':
    main(sys.argv)



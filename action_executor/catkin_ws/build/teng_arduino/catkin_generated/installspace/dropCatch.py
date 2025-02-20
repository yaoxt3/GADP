#!/usr/bin/env python3
# license removed for brevity

import rospy
from teng_arduino.msg import channels
from std_msgs.msg import Int8
import serial
import time
import VADMI_Gripper as GP
import sys


class Catcher:
    def __init__(self):
        self.ready = False
        self.ser1 = serial.Serial(port ='/dev/ttyUSB0', baudrate = 9600)  
        time.sleep(0.5)
        print("Serial connection built")
        
        self.g = GP.Gripper("192.168.0.11")
        self.sub_key = rospy.Subscriber("key",Int8,self.CB_key)
        self.y = [0, 0]

        self.pub = rospy.Publisher('TENG_SIGNALS', channels, queue_size=10)
        self.talker()

    def CB_key(self, data):
        # self.g.drop("off")
        self.ready = True
        self.base = self.y
        print("Base: ", self.base)
           


    def talker(self):
        self.ser1.write("start".encode())
        flag = 0

        while not rospy.is_shutdown():
            if self.ser1.in_waiting > 0:
                if flag == 1:    
                    serData = self.ser1.readline()                    
                    str = serData[0:].decode()
                    x = str.split(",")
                    self.y = [float(i) for i in x]

                    if(self.ready):
                        if max(self.y[0] - self.base[0], self.y[1] - self.base[1] ) > 0.6: 
                            self.g.drop("on")
                            self.ready = False

                    msg = channels()
                    msg.stamp = rospy.Time.now()
                    msg.c = self.y
                    self.pub.publish(msg)


                    

                if flag == 0:
                    a = self.ser1.readline()
                    flag = 1
            






            
def main(args):
    
    rospy.init_node('free_drop_catch_node',anonymous=True)
    C = Catcher()


if __name__ == '__main__':
    main(sys.argv)

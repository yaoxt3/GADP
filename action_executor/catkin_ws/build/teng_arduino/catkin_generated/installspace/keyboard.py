#!/usr/bin/env python3

import getch
import rospy
from std_msgs.msg import String #String message 
from std_msgs.msg import Int8
from pymodbus.client.sync import ModbusTcpClient



################################
# created by yuvaram
#yuvaramsingh94@gmail.com
################################


def keys():
    pub = rospy.Publisher('key',Int8,queue_size=1) # "key" is the publisher name
    rospy.init_node('keypress',anonymous=True)
    rate = rospy.Rate(10)#try removing this line ans see what happens
    while not rospy.is_shutdown():
        k=ord(getch.getch())# this is used to convert the keypress event in the keyboard or joypad , joystick to a ord value
        print("Keys: ", k)# to print on  terminal 
        pub.publish(k)#to publish
        rospy.sleep(0.5)
        #rospy.loginfo(str(k))

        #rate.sleep()

#s=115,e=101,g=103,b=98

if __name__=='__main__':
    try:
	#client = ModbusTcpClient("192.168.0.10", port=502)
        keys()
    except rospy.ROSInterruptException:
        pass



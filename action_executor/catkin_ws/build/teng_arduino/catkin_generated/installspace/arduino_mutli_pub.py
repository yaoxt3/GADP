#!/usr/bin/env python3
# license removed for brevity

import rospy
from teng_arduino.msg import channels
import serial
import time

ser1 = serial.Serial(port ='/dev/ttyUSB0', baudrate = 9600)  
time.sleep(0.5)
print("Serial connection built")

def talker():
    pub = rospy.Publisher('TENG_SIGNALS', channels, queue_size=10)
    rospy.init_node('talker1', anonymous=True)
    ser1.write("start".encode())
    flag = 0

    while not rospy.is_shutdown():
        if ser1.in_waiting > 0:
            if flag == 1:    
                serData = ser1.readline()                    
                str = serData[0:].decode()
                x = str.split(",")
                y = [float(i) for i in x]

                msg = channels()
                msg.stamp = rospy.Time.now()
                msg.c = y
                pub.publish(msg)


                

            if flag == 0:
                a = ser1.readline()
                flag = 1
            

            

if __name__ == '__main__':
    try:
        rospy.loginfo("Serial connection built")

        talker()
    except rospy.ROSInterruptException:
        ser1.write("end".encode())

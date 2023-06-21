#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

# ros msg module
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

class Driver:
    def __init__(self):
        self.Sub_ul = rospy.Subscriber("ultrasonic", Int32MultiArray, self.UL_CB)
        self.Pub_motor = rospy.Publisher("xycar_motor", xycar_motor, queue_size = 1)

        self.ultrasonic = None
        # ultrasonic data callback check
        self.b_ultrasonic = False
        self.left_offset = 1
        self.angle_offset = 5
        self.speed = 50
        self.xycar_msg = xycar_motor()

    def UL_CB(self, ul_data):
        self.ultrasonic = ul_data.data
        CL, CR = self.ultrasonic[1]+self.left_offset, self.ultrasonic[3]
        dif = (CR-CL)*self.angle_offset
        self.drive(angle = dif, speed = self.speed)
        print(dif)


    def drive(self, angle, speed):
        self.xycar_msg.angle = angle
        self.xycar_msg.speed = speed
        self.Pub_motor.publish(self.xycar_msg)
        pass

if __name__ == "__main__":
    drive = Driver()
    rospy.init_node("custom_driver")
    rospy.spin()
    
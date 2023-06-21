#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

# ros msg module
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

class Driver:
    def __init__(self):
        # "ultrasonic" 토픽에 대한 subscriber 생성
        self.Sub_ul = rospy.Subscriber("ultrasonic", Int32MultiArray, self.UL_CB)
        # "xycar_motor" 토픽에 대한 publisher 생성
        self.Pub_motor = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)

        # 초음파 데이터를 저장하는 변수
        self.ultrasonic = None
        # 초음파 데이터 콜백 체크 변수
        self.b_ultrasonic = False
        # 좌측 오프셋과 각도 오프셋, 속도를 설정
        self.left_offset = 1
        self.angle_offset = 5
        self.speed = 50
        # xycar_motor 메시지 객체 생성
        self.xycar_msg = xycar_motor()

    def UL_CB(self, ul_data):
        # 초음파 데이터를 저장
        self.ultrasonic = ul_data.data
        # 좌측과 우측 초음파 거리를 가져온 후 오프셋 적용
        CL, CR = self.ultrasonic[1] + self.left_offset, self.ultrasonic[3]
        # 각도 계산
        dif = (CR - CL) * self.angle_offset
        # 주행 함수 호출하여 각도와 속도 설정
        self.drive(angle=dif, speed=self.speed)
        # 각도 출력
        print(dif)

    def drive(self, angle, speed):
        # xycar_motor 메시지 객체에 각도와 속도 설정
        self.xycar_msg.angle = angle
        self.xycar_msg.speed = speed
        # xycar_motor 토픽에 메시지 발행
        self.Pub_motor.publish(self.xycar_msg)
        pass

if __name__ == "__main__":
    # Driver 클래스의 인스턴스 생성
    drive = Driver()
    # ROS 노드 초기화 및 "custom_driver"라는 이름으로 등록
    rospy.init_node("custom_driver")
    # 노드가 종료될 때까지 유지
    rospy.spin()

    

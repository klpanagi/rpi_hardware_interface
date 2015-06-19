#!/usr/bin/python

from ServoController import ServoController
import rospy

class CameraEffector:
    def __init__(self):
        self.servo_ctrl_ = ServoController()
        self.servo_ctrl_.add_servo('pan', 17)
        self.servo_ctrl_.set_servo_mode('pan')


    def move_servo_at_angle(self, servo_id, angle):
        self.servo_ctrl_.set_angle(servo_id, angle)

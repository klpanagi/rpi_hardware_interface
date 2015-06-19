#!/usr/bin/python

import json
import rospy
import pigpio
import math
import sys
from time import sleep

class ServoController:
    def __init__(self):
        self.gpio_ = pigpio.pi()
        self.pwm_range_ = 1000
        self.pwm_freq_ = 50 # 50 Hz
        self.pwm_period_ = math.pow(10, 6) / self.pwm_freq_
        self.hard_limits_pulse_width_ = [900, 2100]
        print "Range: %s,   Freq: %s,   Period: %s" %(self.pwm_range_, \
            self.pwm_freq_, self.pwm_period_)
        ## Specific to servo
        self.base_positions_ = {'left': 900, 'neutral': 1500, 'right': 2100}
        self.servo_pin_ = {}


    def add_servo(self, _id, pin):
        self.servo_pin_[_id] = pin


    def set_servo_mode(self, servo):
        self.gpio_.set_mode(self.servo_pin_[servo], pigpio.OUTPUT)
        self.gpio_.set_servo_pulsewidth(self.servo_pin_[servo], self.base_positions_['neutral'])

    ##
    #  @brief set the servo at given angle
    #  @param angle Angle in degrees (Absolute)
    #  @param servo Servi index. (String) == 'pan' || 'tilt'
    def set_angle(self, servo, angle):
        pulse_width = self.degrees_to_dutyCycle(angle)
        print "Setting servo position: Pos_Degrees: %s,   Duty_Cycle: %s" \
            %(angle, pulse_width)
        self.gpio_.set_servo_pulsewidth(self.servo_pin_[servo], pulse_width)

    ##
    #  @param old_range [min,max]
    def linear_conversion(self, old_range, new_range, value):
        #a = old_range[1] - old_range[0]
        #b = new_range[1] - new_range[0]
        new_value = 1.0 *(value - old_range[0]) / (old_range[1] - old_range[0])
        new_value  = new_value * (new_range[1] - new_range[0]);
        new_value += new_range[0]
        return new_value

    def dutyCycle_to_degrees(self, duty_cycle):
        new_range = [(-60), 60]
        old_range = [900, 2100]
        degrees = self.linear_conversion(old_range, new_range, duty_cycle)
        return degrees

    def degrees_to_dutyCycle(self, degrees):
        old_range = [-60, 60]
        new_range = [900, 2100]
        duty_cycle = self.linear_conversion(old_range, new_range, degrees)
        return int(duty_cycle)




## ===========================================================================

class CameraEffector:
    def __init__(self):
        self.servo_ctrl_ = ServoController()
        self.servo_ctrl_.add_servo('pan', 17)


        self.servo_ctrl_.set_servo_mode('pan')


    def move_servo_at_angle(self, servo_id, angle):
        self.servo_ctrl_.set_angle(servo_id, angle)



def main():
    #rospy.init_node('camera_effector_node', anonymous=False)
    #rate = rospy.Rate(10) # 25Hz
    effector = CameraEffector()

    for i in range(-60,60,1):
        effector.move_servo_at_angle('pan', i)
        sleep(0.5)


if __name__ == "__main__":
    main()

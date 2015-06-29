#!/usr/bin/python

# Software License Agreement
__version__ = "0.0.1"
__status__ = "Production"
__license__ = "BSD"
__copyright__ = "Copyright (c) 2015, P.A.N.D.O.R.A. Team. All rights reserved."
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of P.A.N.D.O.R.A. Team nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
__author__ = "Konstantinos Panayiotou"
__maintainer__ = "Konstantinos Panayiotou"
__email__ = "klpanagi@gmail.com"


import json
import rospy
import pigpio
import math
import sys



class ServoController:

    ##
    #  Default constructor
    #  @param debug_on True to enable debug mode
    #
    def __init__(self, debug_on):
        self.debug_ = debug_on
        self.gpio_ = pigpio.pi()
        self.registered_servos_ = []
        self.servo_pin_ = {}
        self.servo_pos_ = {}
        self.servo_vel_ = {}
                
        # --------------------- Load parameters ---------------------- #
        left_pos = rospy.get_param( \
            '/servo_controller/servo_params/base_positions/left', 600)
        right_pos = rospy.get_param(
            '/servo_controller/servo_params/base_positions/right', 2400)
        neutral_pos = rospy.get_param( \
            '/servo_controller/servo_params/base_positions/neutral', 1500)

        ## Specific to servo
        self.base_positions_ = {'left': left_pos, \
            'neutral': neutral_pos, 'right': right_pos}

        self.servo_error_degrees_ = rospy.get_param( \
            '/servo_controller/servo_params/error_degrees', 0)

    # ======================================================================= #

    
    ##
    #  Registers_a servo to Servo Cntroller
    #  @param servo_id An id for the servo.
    #  @param pin BCM_GPIO Pin number associated to the servo.
    #
    def register_servo(self, servo_id, pin):
        self.registered_servos_.append(servo_id)
        self.servo_pin_[servo_id] = pin
        self.servo_pos_[servo_id] = None
        self.servo_vel_[servo_id] = None
    # ======================================================================= #


    ##
    #  Enables previously registered servo 
    #  @param servo_id Servo Id.
    #
    def enable_servo(self, servo_id):
        try:
            self.gpio_.set_mode(self.servo_pin_[servo_id], pigpio.OUTPUT)
        except:
            e = sys.exc_info()[0]
            rospy.logfatal(\
                "[Servo-Controller]: Failed to set servo [%s] mode", \
                % servo_id)
            rospy.logfatal(e)
            return False
        finally:
            return True
    # ======================================================================= #


    ##
    #  Returns servo current position in degrees.
    #  @param servo_id Servo Id.
    #
    def get_pos_degrees(self, servo_id):
        return self.servo_pos_[servo_id]
    # ======================================================================= #


    ##
    #  Returns servo current position in degrees.
    #  @param servo_id Servo Id.
    #
    def get_pos_rad(self, servo_id):
        return self.degrees_to_radians(self.servo_pos_[servo_id])
    # ======================================================================= #


    ##
    #  Returns servo current position.
    #  @param servo_id Servo Id.
    #
    def get_vel(self, servo_id):
        return self.servo_vel_[servo_id]
    # ======================================================================= #


    ##
    #  @brief set the servo at given angle
    #  @param degrees (Absolute value)
    #  @param servo_id Servo index. (String) == 'pan' || 'tilt'
    #
    def set_pos_degrees(self, servo_id, degrees):
        # --- Perform error correction too --- #
        degrees_no_error = degrees - self.servo_error_degrees_
        pulse_width = self.degrees_to_dutyCycle(degrees_no_error)

        try:
            self.gpio_.set_servo_pulsewidth(self.servo_pin_[servo_id], \
                pulse_width)
        except:
            e = sys.exc_info()[0]
            rospy.logfatal("[Servo-Controller]: Failed to set pulse_width")
            rospy.logfatal(e)
            return False
        finally:
            if self.debug_:
                rospy.loginfo( \
                    "Setting servo [%s] position (Absolute Values):" + \
                    " Pos_Degrees: %s,   Duty_Cycle: %s" \
                    %(servo_id, degrees, pulse_width) )
            self.servo_pos_[servo_id] = degrees
            return True
    # ======================================================================= #


    ##
    #  Disable servo. Set pwm off.
    #
    def disable_servo(self, servo_id):
        off = 0;
        pin = self.servo_pin_[servo_id]
        try:
            self.gpio_.set_servo_pulsewidth(pin, off)
        except:
            e = sys.exc_info()[0]
            rospy.logfatal("[Servo-Controller]: Failed to set pulse_width")
            rospy.logfatal(e)
            return False
        finally:
            rospy.logwarn("[Servo-Controller]: Disabled servo --> %s", servo_id)
            return True
    # ======================================================================= #


    ## 
    #  Linear conversion function
    #  @param old_range [min,max]
    #  @param new_range [min,max]
    #  @param value Value to perform linear conversion
    #
    def linear_conversion(self, old_range, new_range, value):
        #a = old_range[1] - old_range[0]
        #b = new_range[1] - new_range[0]
        new_value = 1.0 *(value - old_range[0]) / (old_range[1] - old_range[0])
        new_value  = new_value * (new_range[1] - new_range[0]);
        new_value += new_range[0]
        return new_value
    # ======================================================================= #


    ##
    #  Translattes PWM duty_cycle command to degrees.
    #  @param duty_cycle
    #
    def dutyCycle_to_degrees(self, duty_cycle):
        new_range = [-90, 90]
        old_range = [600, 2400]
        degrees = self.linear_conversion(old_range, new_range, duty_cycle)
        return degrees
    # ======================================================================= #

    
    ##
    #  Translates degrees to PWM duty cycle to command the servo
    #  @param degrees
    #
    def degrees_to_dutyCycle(self, degrees):
        old_range = [-90, 90]
        new_range = [600, 2400]
        duty_cycle = self.linear_conversion(old_range, new_range, degrees)
        # TODO -- Handle with possible error on conversion
        #print "Duty Cucle: %s" % duty_cycle
        return int(duty_cycle) # Return an integer
    # ======================================================================= #


    ##
    #  Converts angle values to radians
    #  @param degrees 
    #
    def degrees_to_radians(self, degrees):
        return (1.0 * degrees * math.pi / 180)
    # ======================================================================= #


    ##
    #  Converts radian values to degrees
    #  @param rad
    #
    def radians_to_degrees(self, rad):
        return (rad * 180 / math.pi)
    # ======================================================================= #


    ##
    #  Default destructor
    #
    def __del__(self):
        for servo_id in self.registered_servos_:
            self.disable_servo(servo_id)
    # ======================================================================= #




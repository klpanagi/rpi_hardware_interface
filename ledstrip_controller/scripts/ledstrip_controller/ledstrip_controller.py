#!/usr/bin/python

# Software License Agreement
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

__author__ = "George Kouros"
__maintainer__ = "George Kouros"
__email__ = "gkourosg@yahoo.gr"


import rospy
import pigpio
import json
from state_manager_msgs.msg import RobotModeMsg
from std_msgs.msg import Bool, Float64
import roslib
roslib.load_manifest("ledstrip_controller")

# Dynamic Reconfigure Server
from dynamic_reconfigure.server import Server as DynRecServer

#from ledstrip_controller import ledstrip_controllerConfig as Config

class LedstripController():

    ##
    #  Default constructor
    #
    def __init__(self):
        self.led_state_ = False
        self.brightness_ = 1.0
        self.pwm_duty_ = 255

        self.pwm_range_ = [0, 255]
        self.bright_range_ = [0, 1]
        self.gpio_handler_ = pigpio.pi()

        # ----< Load parameters from parameter server >---- #
        self.ledstrip_gpio_ = rospy.get_param( "~led_gpio_pin", 26)

        self.topic_cmd_ = rospy.get_param( "~subscribed_topics/ledstrip_cmd", \
                "/kinect/ledstrip/state_cmd")

        self.topic_bright_ = rospy.get_param( "~subscribed_topics/brightness", \
                "/kinect/ledstrip/brightness")

        self.topic_state_client_ = rospy.get_param( \
                "~subscribed_topics/state_client", "/robot/state/clients")

        self.topic_state_ = rospy.get_param( \
                "~published_topics/ledstrip_state", "/kinect/ledstrip/state")

        self.pub_rate_ = rospy.get_param("~pub_rate", 1)
        # ------------------------------------------------- #

        # ---------------------< Subscribers >---------------------- #
        rospy.Subscriber(self.topic_cmd_, Bool, self.__command_callback)

        rospy.Subscriber(self.topic_state_client_, RobotModeMsg, \
                self.__state_change_callback)

        rospy.Subscriber(self.topic_bright_, Float64, 
                self.__brightness_callback)
        # ---------------------------------------------------------- #

        # ----------------------< Publishers >---------------------- #
        self.state_pub_ = rospy.Publisher(self.topic_state_, Float64, 
                queue_size=1)
        # ---------------------------------------------------------- #

        # -----------< Initiate gpio to control ledstrip > --------- #
        self.gpio_handler_.set_mode(self.ledstrip_gpio_, 1)
        #self.gpio_handler_.write(self.ledstrip_gpio_, 0)
        self.__led_control(0)
        # ---------------------------------------------------------- #

        #self.dyn_rec_server_ = DynRecServer(Config, self.reconfigure)
        
        # Start Publishing timer 
        rospy.Timer(rospy.Duration(float(1/self.pub_rate_)), \
                self.__publish_led_state_callback, oneshot=False)
    # ======================================================================= #


    ##
    #  Callback for Dynamic Reconfigure Server
    #
    #def reconfigure(self, config, level):
        #self.brightness_ = config['brightness']
        #return config


    ##
    #  Default destructor
    ##
    def __del__(self):
        self.__led_control(0)
        del self.gpio_handler_
    # ======================================================================= #


    ##
    #  TODO
    ##
    def __state_change_callback(self, data):
        if data.mode == data.MODE_IDENTIFICATION or \
                data.mode == data.MODE_SENSOR_HOLD:
            self.__led_control(1)
        else:
            self.__led_control(0)
    # ======================================================================= #


    ##
    #  TODO
    ##
    def __brightness_callback(self, msg):
        bright_val = msg.data
        if bright_val > 1.0:
            bright_val = 1.0

        rospy.loginfo("[Ledstrip-Controller]: Setting brightness ---> %s"\
                % bright_val)
        self.__set_brightness(bright_val)
    # ======================================================================= #


    ##
    #  TODO
    ##
    def __command_callback(self, msg):
        self.led_state = msg.data
        #rospy.loginfo("Turning Led : %s" % msg.data)
        self.__led_control(msg.data)
    # ======================================================================= #


    ##
    #  TODO
    ##
    def __publish_led_state_callback(self, event):
        msg = Float64(self.brightness_)
        self.state_pub_.publish(msg)
    # ======================================================================= #


    ##
    #  TODO
    ##
    def __set_brightness(self, bright_val):         
        self.pwm_duty_ = self.linear_conversion(self.bright_range_,
                self.pwm_range_, bright_val)
        if self.__led_control(1):
            self.brightness_ = bright_val
    # ======================================================================= #


    ##
    #  TODO
    ##
    def __led_control(self, value):
        try:
            if value == 1:
                self.gpio_handler_.set_PWM_dutycycle(self.ledstrip_gpio_, 
                        self.pwm_duty_)
            else:
                self.gpio_handler_.set_PWM_dutycycle(self.ledstrip_gpio_, 0)
        except:
            e = sys.exc_info()[0]
            rospy.logfatal(\
                    "[Ledstrip-Controller]: Failed to set Ledstrip State")
            rospy.logfatal(e)
            return False
        finally:
            self.led_state_ = value
            rospy.loginfo("[Ledstrip-Controller]: Setting ledstrip State ---> %s"\
                    % value)
            return True
    # ======================================================================= #


    ## 
    #  Linear conversion function
    #  @param old_range [min,max]
    #  @param new_range [min,max]
    #  @param value Value to perform linear conversion
    ##
    def linear_conversion(self, old_range, new_range, value):
        #a = old_range[1] - old_range[0]
        #b = new_range[1] - new_range[0]
        new_value = 1.0 *(value - old_range[0]) / (old_range[1] - old_range[0])
        new_value  = new_value * (new_range[1] - new_range[0]);
        new_value += new_range[0]
        return new_value
    # ======================================================================= #



def main():
    rospy.init_node("ledstrip_controller")
    ledstrip_ctrl = LedstripController()
    rospy.spin()


if __name__ == "__main__":
    main()

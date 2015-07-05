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
from std_msgs.msg import Bool


class LedstripController():

  ##
  #  Default constructor
  #
  def __init__(self):

    self.led_state_ = False
    self.gpio_handler_ = pigpio.pi()

    # ----< Load parameters from parameter server >---- #
    self.ledstrip_gpio_ = rospy.get_param( "~led_gpio_pin", 26)

    self.topic_cmd_ = rospy.get_param( "~subscribed_topics/ledstrip_cmd", \
        "/kinect/ledstrip/state_cmd")

    self.topic_state_client_ = rospy.get_param( \
        "~subscribed_topics/state_client", "/robot/state/clients")

    self.topic_state_ = rospy.get_param( \
        "~published_topics/ledstrip_state", "/ledstrip/state")

    self.pub_rate_ = rospy.get_param("~pub_rate", 1)
    # ------------------------------------------------- #

    # ---------------------< Subscribers >---------------------- #
    rospy.Subscriber(self.topic_cmd_, Bool, self.command_callback)
    rospy.Subscriber(self.topic_state_client_, RobotModeMsg, \
            self.state_change_callback)
    # ---------------------------------------------------------- #

    # ----------------------< Publishers >---------------------- #
    self.state_pub_ = rospy.Publisher(self.topic_state_, Bool, queue_size=1)
    # ---------------------------------------------------------- #

    # -----------< Initiate gpio to control ledstrip > --------- #
    self.gpio_handler_.set_mode(self.ledstrip_gpio_, 1)
    self.gpio_handler_.write(self.ledstrip_gpio_, 0)
    # ---------------------------------------------------------- #

    # Start Publishing timer 
    rospy.Timer(rospy.Duration(float(1/self.pub_rate_)), \
            self.publish_led_state_callback, oneshot=False)


  ##
  #  
  def state_change_callback(self, data):
    if data.mode == data.MODE_IDENTIFICATION:
      self.led_control(1)
    else:
      self.led_control(0)


  def command_callback(self, msg):
      self.led_state = msg.data
      rospy.loginfo("Turning Led : %s" % msg.data)
      self.led_control(msg.data)


  def publish_led_state_callback(self, event):
    msg = Bool(self.led_state_)
    self.state_pub_.publish(msg)


  def led_control(self, value):
      try:
        self.gpio_handler_.write(self.ledstrip_gpio_, value)
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



def main():
  rospy.init_node("ledstrip_controller")
  ledstrip_ctrl = LedstripController()
  rospy.spin()


if __name__ == "__main__":
    main()

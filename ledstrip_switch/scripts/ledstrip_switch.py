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
import RPi.GPIO as GPIO
from state_manager_msgs import RobotModeMsg
from std_msgs import Bool


class LedstripSwitch():

  def __init__(self):
    self.ledstrip_gpio = 26
    GPIO.setup(ledstrip_gpio, GPIO.OUT)
    self.led_state = False
    GPIO.output(self.ledstrip_gpio, self.led_state)

    rospy.Subscriber("/ledstrip/command", Bool, command_callback)
    rospy.Subscriber("/robot/state/clients", RobotModeMsg, state_callback)
    state_pub = rospy.Publisher("/ledstrip/state", Bool, queue_size=1)

    rospy.Timer(rospy.Duration(1.0), self.publish_led_state_callback, oneshot=False)
    rospy.spin()

  def state_callback(self, data):
    if data.mode == data.MODE_IDENTIFICATION:
      self.led_state = True
    else:
      self.lde_state = False
    GPIO.output(self.ledstrip_gpio, self.led_state)

  def command_callback(self, data):
      self.led_state = data
      GPIO.output(self.ledstrip_gpio, data)

  def publish_led_state_callback(self, event):
    led_state_msg = Bool(self.led_state)
    state_pub.publish(led_state_msg)

if __name__ == "__main__":
  rospy.init_node("ledstrip_switch_node")
  ledStripSwitch = LedStripSwitch()

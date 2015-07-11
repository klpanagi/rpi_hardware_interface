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



from ServoController import ServoController
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import math



class CameraEffector:
    ##
    #  Default Constructor
    #
    def __init__(self):
        self.debug_ = 0 
        self.acceptable_cmd_error_ = 0.1
        self.servo_ctrl_ = ServoController(0) # 1 == Enable debug mode
        self.joint_state_msg_ = JointState()
        self.joint_name_ = {}
        self.subscribers_ = {}
        self.pos_limits_ = {}
        self.offsets_ = {}
        self.camera_orientation_ = {}
        self.camera_orientation_['pan'] = None
        self.camera_orientation_['tilt'] = None

        # --------------------- Load parameters ---------------------- #
        self.joint_states_topic_ = rospy.get_param( \
            '~published_topics/camera_effector_joint_states', \
            '/rpi2/camera_effector/joint_states')
        self.joint_name_['pan'] = rospy.get_param( \
            '~pan_joint/joint_name', 'camera_effector_pan_joint')
        self.joint_name_['tilt'] = rospy.get_param( \
            '~tilt_joint/joint_name', 'camera_effector_tilt_joint')
        self.frame_id_ = rospy.get_param( \
            '~camera_effector_urdf/camera_effector_frame', \
            '/camera_effector_frame')
        self.subscribers_['pan_command'] = rospy.get_param( \
            '~subscribed_topics/pan_command', '/camera_effector/pan_command')
        self.subscribers_['tilt_command'] = rospy.get_param( \
            '~subscribed_topics/tilt_command', '/camera_effector/tilt_command')
                
        pan_servo_pin = rospy.get_param('~pan_joint/servo/pin', 17)
        tilt_servo_pin = rospy.get_param('~tilt_joint/servo/pin', 18)
        self.offsets_['pan'] = rospy.get_param(
            '~pan_joint/offset', 0)
        self.offsets_['tilt'] = rospy.get_param(
            '~tilt_joint/offset', -0.1920)  # 11 degrees

        max_pos = rospy.get_param('~tilt_joint/limits/max', 1.3962)
        min_pos = rospy.get_param('~tilt_joint/limits/min', -1.3962)
        self.pos_limits_['tilt'] = {'max': max_pos, 'min': min_pos}

        max_pos = rospy.get_param('~pan_joint/limits/max', 0.6)
        min_pos = rospy.get_param('~pan_joint/limits/min', -0.6)
        self.pos_limits_['pan'] = {'max': max_pos, 'min': min_pos}
        # ------------------------------------------------------------ #

        # ---------------------- Subscribers ------------------------- #
        rospy.Subscriber(self.subscribers_['pan_command'], Float64, \
            self.pan_command_callback)
        rospy.Subscriber(self.subscribers_['tilt_command'], Float64, \
            self.tilt_command_callback)
        # ------------------------------------------------------------ #

        # ---------------------- Publishers -------------------------- #
        self.joint_state_pub_ = rospy.Publisher(self.joint_states_topic_, \
            JointState, queue_size = 1)
        # ------------------------------------------------------------ #

        # ---------------- Configure PAN & TILT Servos --------------- #
        self.servo_ctrl_.register_servo('pan', pan_servo_pin)
        self.servo_ctrl_.enable_servo('pan')
        self.set_servo_pos('pan', 0)
        self.servo_ctrl_.register_servo('tilt', tilt_servo_pin)
        self.servo_ctrl_.enable_servo('tilt')
        self.set_servo_pos('tilt', 0)
        # ------------------------------------------------------------ #
        
        # ------------- Construct publishing ROS message ------------- #
        self.joint_state_msg_.header.stamp = rospy.Time.now()
        self.joint_state_msg_.header.frame_id = self.frame_id_
        self.joint_state_msg_.name.append(self.joint_name_['pan'])
        self.joint_state_msg_.name.append(self.joint_name_['tilt'])
        self.joint_state_msg_.position.append(0)
        self.joint_state_msg_.position.append(0)
        # ------------------------------------------------------------ #

        rospy.logwarn("[Camera-Effector]: Kinematics limits [Tilt]: " + \
            "{min --> %s,  max --> %s}", self.pos_limits_['tilt']['min'], \
            self.pos_limits_['tilt']['max'])
        rospy.logwarn("[Camera-Effector]: Kinematics limits [Pan]: " + \
            "{min --> %s,  max --> %s}", self.pos_limits_['pan']['min'], \
            self.pos_limits_['pan']['max'])
    # ======================================================================= #


    ##
    #  Pan Command callback
    #  Commands Pan servo to move at given command-value.
    #  @param command std_msgs/Float64 command message.
    #
    def pan_command_callback(self, command):
        self.set_servo_pos('pan', command.data)
        if self.debug_:
            rospy.loginfo("Pan command: %s" % command.data)
    # ======================================================================= #


    ##
    #  Tilt Command callback
    #  Commands Tilt servo to move at given command-value.
    #  @param command std_msgs/Float64 command message.
    #
    def tilt_command_callback(self, command):
        self.set_servo_pos('tilt', command.data)
        if self.debug_:
            rospy.loginfo("Tilt command: %s" % command.data)
    # ======================================================================= #



    ##
    #  Command servo to move at given angle
    #  @param servo_id Servo id to move
    #  @param angle Move to angle==<angle>
    #
    def set_servo_pos(self, servo_id, rad):
        min_accept = self.pos_limits_[servo_id]['min'] * \
            self.acceptable_cmd_error_ + self.pos_limits_[servo_id]['min']
        max_accept = self.pos_limits_[servo_id]['max'] * \
            self.acceptable_cmd_error_ + self.pos_limits_[servo_id]['max']

        # ----- Check off-limits command ---- #
        if rad < min_accept or rad > max_accept:
            # Do not publish. Invalid command
            rospy.logwarn("Command [%s <--> rad] out of bounds"\
                + "  ---> Limits: {min: %s, max: %s}", rad, \
                self.pos_limits_[servo_id]['min'], \
                self.pos_limits_[servo_id]['max'])
            rospy.logfatal("Command is not acceptable!! Staying idle")
            return 0

        elif rad > self.pos_limits_[servo_id]['max']:
            rospy.logwarn("Command [%s <--> rad] out of bounds"\
                + "  ---> Limits: {min: %s, max: %s}. " + \
                "Commanding at max position", rad, \
                self.pos_limits_[servo_id]['min'], \
                self.pos_limits_[servo_id]['max'])

            rad = self.pos_limits_[servo_id]['max']

        elif rad < self.pos_limits_[servo_id]['min']:
            rospy.logwarn("Command [%s <--> rad] out of bounds"\
                + "  ---> Limits: {min: %s, max: %s}. " + \
                "Commanding at min position", rad, \
                self.pos_limits_[servo_id]['min'], \
                self.pos_limits_[servo_id]['max'])
            rad = self.pos_limits_[servo_id]['min']
        # ----------------------------------- #

        # ---< Used mainly for debug purposes >--- #
        degrees = self.servo_ctrl_.radians_to_degrees(rad)
        if self.debug_:
            rospy.loginfo(\
                "[%s servo]: Commanding camera at angle: %s radians, %s degrees" % \
                (servo_id, rad, degrees))
        rad -= self.offsets_[servo_id]

        #  ** Use the transformatio function to transform camera joint angle to 
        #  ** servo joint angle.
        if servo_id == 'tilt':
            pass
            #rad = self.camera_to_servo_transform(rad)

        degrees = self.servo_ctrl_.radians_to_degrees(rad)
        if self.debug_:
            rospy.loginfo(\
                "[%s servo]: Commanding servo at angle: %s degrees , %s radians" % \
                (servo_id, degrees, rad))
        # --------------------------------------- #
        self.servo_ctrl_.set_pos_degrees(servo_id, degrees)
        return 1
    # ======================================================================= #


    ##
    #  Read from ServoController and Publish
    #
    def read(self):
        self.update_camera_orientation()
        current_time_stamp = rospy.Time.now()
        # position[0]==<position['pan']> , position[1]==<position['tilt']>
        self.joint_state_msg_.position[0] = self.camera_orientation_['pan']
        self.joint_state_msg_.position[1] = self.camera_orientation_['tilt']
        self.joint_state_msg_.header.stamp = current_time_stamp
        self.joint_state_pub_.publish(self.joint_state_msg_)
    # ======================================================================= #


    ##
    #  Update Camera Orientation 
    #
    def update_camera_orientation(self):
        servo_pos = {}
        servo_pos['pan'] = self.servo_ctrl_.get_pos_rad('pan')
        servo_pos['tilt'] = self.servo_ctrl_.get_pos_rad('tilt')

        self.camera_orientation_['pan'] = servo_pos['pan'] + \
          self.offsets_['pan']

        self.camera_orientation_['tilt'] = servo_pos['tilt'] + \
          self.offsets_['tilt']
    # ======================================================================= #


    ##
    #  Default destructor
    #
    def __del__(self):
        del self.servo_ctrl_
    # ======================================================================= #


    ##
    #  Transforms from camera joint to servo joint angle.
    #  @param angle Camera joint angle value
    #
    def camera_to_servo_transform(self, angle):
        # ---< Constant facotrs >--- #
        A = 32  
        B = 13
        r1 = 10
        r2 = 30
        r3 = 12
        # -------------------------- #

        y2 = 1.0 * math.cos(angle) * r3
        x2 = 1.0 * math.sin(angle) * r3
        y1 = B - y2
        x1 = A + x2

        temp = 1.0 * x1 / y1
        fi_angle = math.atan(temp)

        p = 1.0 * x1 / math.sin(fi_angle)

        temp = 1.0 * (1.0 * r1 * r1 + 1.0 * p * p - 1.0 * r2 * r2) \
            / (2.0 * r1 * p)
        r1p_angle = math.acos(temp)

        theta = fi_angle - r1p_angle
        return theta
    # ========================================================================


#!/usr/bin/env python

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


from sensor_msgs.msg import Image
import picamera
import io
import sys
import timeit
import time
import json
import rospy
import thread
from threading import Thread
#import picamera.array
import warnings

#warnings.filterwarnings('warn', category=DeprecationWarning)

class PiCameraRosWrapper:
    ## TODO Load parameters from parameter server!!!
    ## TODO Dynamic Reconfigure

    #=======================================================================
    ## Default constructor
    def __init__(self):
        self.thread_ = None

        # ------------------- Load Parameters ------------------ #
        self.res_width_ = rospy.get_param('~width', 640)
        self.res_height_ = rospy.get_param('~height', 480)
        self.framerate_ = rospy.get_param('~framerate', 30)
        self.image_format_ = rospy.get_param('~image_format', \
            'rgb')
        self.use_video_port_ = rospy.get_param('~use_video_port',\
            True)
        self.exposure_mode_ = rospy.get_param('~exposure_mode', \
            'fixedfps')
        self.image_topic_ = rospy.get_param('~published_topics/image', \
            '/rpi2/pi_camera/image')
        self.optical_frame_ = rospy.get_param(\
            '~pi_camera_urdf/optical_frame', '/pi_camera_optical_frame')
        # ------------------------------------------------------ #

        #self.flash_mode_ = 'off'
        #self.shutter_speed_ = 30000 # us
        #self.led_state_ = 'on'
       
        #self.pub_thread_ = Thread(
        #    target=self.publish)

        # Initiate camera module
        self.camera_ = picamera.PiCamera()
        self.set_resolution(self.res_width_, self.res_height_)
        self.set_framerate(self.framerate_)
        self.set_exposure_mode(self.exposure_mode_)
        #self.set_shutter_speed(self.shutter_speed_)
        #self.set_flash_mode(self.flash_mode_)
        #self.set_led_state(self.led_state_)

        print "[PiCamera]: Waiting for camera to warmup for 2 seconds"
        for i in range(2, 0, -1):
            print "[PiCamera]: %s" %i
            time.sleep(1)

        rospy.loginfo("Shutter Speed: [%s]" % self.camera_.shutter_speed)
        rospy.loginfo("Exposure Speed: [%s]" % self.camera_.exposure_speed)
        rospy.loginfo("Exposure Mode: [%s]" % self.camera_.exposure_mode)
        rospy.loginfo("Resolution Width: [%s]" %self.res_width_)
        rospy.loginfo("Resolution Height: [%s]" %self.res_height_)

        self.image_pub_ = rospy.Publisher(self.image_topic_, Image, \
            queue_size=1)

        self.image_msg_ = Image()
        self.image_msg_.height = self.res_height_
        self.image_msg_.width = self.res_width_
        self.image_msg_.encoding = 'rgb8'
        self.image_msg_.step = self.res_width_ * 3
        self.image_msg_.header.frame_id = self.optical_frame_
    #=======================================================================


    ## Sets camera image capture framerate (fps)
    # @param framerate Camera capture rate in fps
    def set_framerate(self, framerate):
        self.camera_.framerate = (framerate, 1)
    #=======================================================================


    ## Sets camera resolution
    # @param width Image width (e.g 640)
    # @param height Image height (e.g. 480)
    #
    def set_resolution(self, width, height):
        self.camera_.resolution = (width, height)
    #=======================================================================


    ## Sets camera exposure mode 
    #  @param mode Exposure mode.
    #
    def set_exposure_mode(self, mode):
        self.camera_.exposure_mode = mode

    def set_shutter_speed(self, speed):
        self.camera_.shutter_speed = speed # us
    
    def set_flash_mode(self, mode):
        self.camera_.flash_mode = mode

    def set_led_state(self, state):
        if state == 'on':
            self.camera_.led = True
            return True
        elif state == 'off':
            self.camera_.led = False
            return True
        else:
            rospy.logwarn("[PiCamera]: Invalid Argument")
            return False

    ##
    #   Used in order to publish an image frame to the releavant topic
    #   @param frame Contains image values
    #   @return Success index -- Boolean
    #
    def publish(self, frame):
        #frame = stream.getvalue()
        if len(frame) == self.res_height_ * self.res_width_ * 3:
            self.image_msg_.data = frame
            self.image_msg_.header.stamp = rospy.Time.now()
            self.image_pub_.publish(self.image_msg_)
            return True
        else:
            rospy.logerr("[PiCamera]: Wrong frame size [%s]", len(frame))
            return False
        
    ##
    #   Asyncronous execution
    #   @return None
    #
    def run_continuous_async(self):
        try:
            self.thread_ = Thread(\
                target=self.run_continuous_threaded, args=()) 
            self.thread_.start()
        except:
            rospy.logfatal('Error on initializing async thread')
            return 0
        finally:
            return 1


    ##
    #   This implemetation uses a thread to handle frame capturing
    #   @return None
    #
    def run_continuous_threaded(self):
        rawCapture = io.BytesIO()
        startT = timeit.default_timer()
        frame = None
        
        for foo in self.camera_.capture_continuous(rawCapture, \
                self.image_format_, use_video_port=self.use_video_port_):
            endT = timeit.default_timer()
            elapsedT = endT - startT
            startT = endT
            thread.start_new_thread(self.publish, (rawCapture.getvalue(), ))
            rawCapture.truncate(0)
            rawCapture.seek(0)
        return


    def run_continuous(self):
        rawCapture = io.BytesIO()
        startT = timeit.default_timer()
        frame = None
        
        for foo in self.camera_.capture_continuous(rawCapture, \
                self.image_format_, use_video_port=self.use_video_port_):
            endT = timeit.default_timer()
            elapsedT = endT - startT
            startT = endT
            #rospy.logwarn("Elapsed Time: [%s]", elapsedT)

            frame = rawCapture.getvalue()
            if len(frame) == self.res_height_ * self.res_width_ * 3:
                self.image_msg_.data = frame
                self.image_pub_.publish(self.image_msg_)
            else:
                rospy.logerr("[PiCamera]: Wrong frame size [%s]", len(frame))

            #rospy.loginfo("Elapsed Time: [%s]", elapsedT)
            rawCapture.truncate(0)
            rawCapture.seek(0)


    def stream(self):
        rawCapture = io.BytesIO()
        
        while True:
            start_time = timeit.default_timer()
            try: 
                yield rawCapture
            except:
                e = sys.exc_info()[0]
                rospy.logwarn("[PiCamera]: Failed to fetch current frame!")
            end_time = timeit.default_timer()
            elapsed_time = end_time - start_time
            start_time = end_time
            thread.start_new_thread(self.publish, (rawCapture.getvalue(), ))
            rawCapture.truncate(0)
            rawCapture.seek(0)
            
            


    def run_sequential(self):
        self.camera_.capture_sequence(self.stream(), self.image_format_, self.use_video_port_)






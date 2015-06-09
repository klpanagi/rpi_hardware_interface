#/usr/bin/env python

from sensor_msgs.msg import Image
import picamera
import io
import sys
import timeit
import time
import json
import rospy
#import picamera.array

class PiCameraRosWrapper:
    ## TODO Load parameters from parameter server!!!
    ## TODO Dynamic Reconfigure

    #=======================================================================
    ## Default constructor
    def __init__(self):
        self.res_width_ = 640
        self.res_height_ = 480
        self.framerate_ = 20
        self.image_format_ = 'rgb'
        self.use_video_port_ = True
        self.exposure_mode_ = 'fixedfps'
        #self.flash_mode_ = 'off'
        #self.shutter_speed_ = 30000 # us
        #self.led_state_ = 'on'
        
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

        self.image_pub_ = rospy.Publisher('rpi2/pi_camera/image', Image, queue_size=100)

        self.image_msg_ = Image()
        self.image_msg_.height = self.res_height_
        self.image_msg_.width = self.res_width_
        self.image_msg_.encoding = 'rgb8'
        self.image_msg_.step = self.res_width_ * 3


    #=======================================================================


    ## Sets camera image capture framerate (fps)
    # @param framerate Camera capture rate in fps
    def set_framerate(self, framerate):
        self.camera_.framerate = framerate
    #=======================================================================


    ## Sets camera resolution
    # @param width Image width (e.g 640)
    # @param height Image height (e.g. 480)
    def set_resolution(self, width, height):
        self.camera_.resolution = (width, height)
    #=======================================================================

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

    def run_continuous(self):
        rawCapture = io.BytesIO()
        startT = timeit.default_timer()
        frame = None
        
        for frame in self.camera_.capture_continuous(rawCapture, \
                self.image_format_, self.use_video_port_):
            endT = timeit.default_timer()
            elapsedT = endT - startT
            startT = endT
            #print "Elapsed Time: [%s]" % elapsedT

            frame = rawCapture.getvalue()
            if len(frame) == self.res_height_ * self.res_width_ * 3:
                self.image_msg_.data = frame
                self.image_pub_.publish(self.image_msg_)
            else:
                rospy.logerr("[PiCamera]: Wrong frame size [%s]", len(frame))
            

            #rospy.loginfo("Elapsed Time: [%s]", elapsed_time)
            rawCapture.truncate(0)
            rawCapture.seek(0)

    def stream(self):
        stream = io.BytesIO()
        
        while True:
            start_time = timeit.default_timer()
            try: 
                yield stream
            except:
                e = sys.exc_info()[0]
                rospy.logwarn("[PiCamera]: Failed to fetch current frame!")
            end_time = timeit.default_timer()
            elapsed_time = end_time - start_time
            frame = stream.getvalue()
            if len(frame) == self.res_height_ * self.res_width_ * 3:
                self.image_msg_.data = frame
                self.image_pub_.publish(self.image_msg_)
            else:
                rospy.logerr("[PiCamera]: Wrong frame size [%s]", len(frame))

            #rospy.loginfo("Elapsed Time: [%s]", elapsed_time)
            stream.truncate(0)
            stream.seek(0)
            
            


    def run_sequential(self):
        self.camera_.capture_sequence(self.stream(), self.image_format_, self.use_video_port_)






#!/usr/bin/python

import picamera
import picamera.array
import time
import json
import io
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image



## ---- RPI Camera Interfacing Class
class PiCameraInterface:
    def __init__(self):
        self.res_width_ = 640
        self.res_height_ = 480
        self.framerate_ = 30
        self.image_format_ = 'rgb'
        self.use_video_port_ = True

        self.camera_ = picamera.PiCamera()
        self.image_stream_ = io.BytesIO()
    
        self.set_camera_params()
        self.rgb_array_ = picamera.array.PiRGBArray(self.camera_)
        #self.image_ = picamera.array.PiRGBArray(self.camera_)

    def set_camera_params(self):
        self.set_resolution(self.res_width_, self.res_height_)
        self.set_framerate(self.framerate_)

    def preview(self, time):
        self.camera_.start_preview()
        time.sleep(time)
        self.camera_.stop_preview()


    def startPreview(self):
        self.camera_.start_preview()


    def stopPreview(self):
        self.camera_.stop_preview()


    def set_resolution(self, width, height):
        self.camera_.resolution = (width, height)


    def set_framerate(self, framerate):
        self.camera_.framerate = framerate


    def closeDevice(self):
        self.camera_.close()
    
    def stream(self):
        stream = io.BytesIO()

        # This returns the stream for the camera to capture to
        yield stream
        # Finally, reset the stream for the next capture

        self.image_stream_ = stream

    def get_image_from_stream(self):
        return self.image_stream_.getvalue()


    def get_image_from_rgb_array(self):
        return self.rgb_array_.array
        

    def capture_toStream(self):
        # Create an in-memory stream
        self.image_stream_.truncate(0)
        self.camera_.capture(self.image_stream_, self.image_format_, self.use_video_port_)
        print len(self.image_stream_.getvalue())


    def capture_toArray(self):
        # Emptying the rgb_array_ with truncate(0) between captures:
        self.rgb_array_.truncate(0)
        # Create an in-memory stream
        self.camera_.capture(self.rgb_array_, self.image_format_, self.use_video_port_)

    def capture_sequence_toStream(self):
        self.camera_.capture_sequence(self.stream(), self.image_format_, self.use_video_port_)
        print len(self.image_stream_.getvalue())

    def capture_sequence_toArray(self):
        self.camera_.capture_sequence(self.rgb_array_, self.image_format_, self.use_video_port_)

    # Captures and stores in a file!!
    def capture_image_file(self, name, form):
        fileName = name
        self.camera_.capture(fileName, form, True)

    def __del__(self):
        self.camera_.close()
        del self.camera_
#------------------------------------------------------------------


class PiCamera:
    def __init__(self):
        self.piCamera_ = PiCameraInterface()
        #pub = rospy.Publisher('picamera', String, queue_size=100)
        self.image_pub_ = rospy.Publisher('picamera/image', Image, queue_size=100)
        #hello_str = "Hello i am picamera: [%s]" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        
    def run(self):
        #self.piCamera_.capture_toArrvay()
        #self.piCamera_.capture_toStream()
        self.piCamera_.capture_sequence_toStream()
        image = self.piCamera_.get_image_from_stream()
        #image = self.piCamera_.get_image_from_rgb_array()
        #print len(image)
        msg = Image()
        msg.height = self.piCamera_.res_height_
        msg.width = self.piCamera_.res_width_
        msg.encoding = 'rgb8'
        msg.step = self.piCamera_.res_width_ * 3
        msg.data = image
        print "Image Data length: [%s]" %len(msg.data)
        self.image_pub_.publish(msg)


    def __del__(self):
        self.piCamera_.closeDevice()
        del self.piCamera_



def main():
    piCamera = PiCamera()
    rospy.init_node('picamera', anonymous=False)
    rate = rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown():
        rate.sleep()
        piCamera.run()


    #for i in range(0, 100):
        #start_time = time.time()
        #piCamera.capture_image_file('test%s'%i, 'jpeg')
        #piCamera.capture_toStream()
        #piCamera.capture_sequence_toStream()
        #stop_time = time.time()
        #exec_time = stop_time - start_time
        #print "CaptureImage Exec time: %s"%exec_time



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

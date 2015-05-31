#!/usr/bin/python

import time
import json
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from pi_camera import pi_camera_interface


class PiCamera:
    def __init__(self):
        self.piCamera_ = pi_camera_interface.PiCameraInterface()
        #pub = rospy.Publisher('picamera', String, queue_size=100)
        self.image_pub_ = rospy.Publisher('picamera/image', Image, queue_size=100)
        #hello_str = "Hello i am picamera: [%s]" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        
    def run(self):
        #self.piCamera_.capture_sequence_toStream()
        image = self.piCamera_.get_image_from_stream()
        print len(image)
        msg = Image()
        msg.height = self.piCamera_.res_height_
        msg.width = self.piCamera_.res_width_
        msg.encoding = 'rgb8'
        msg.step = self.piCamera_.res_width_ * 3
        msg.data = image
        self.image_pub_.publish(msg)


    def __del__(self):
        self.piCamera_.closeDevice()
        del self.piCamera_



def main():
    piCamera = PiCamera()
    rospy.init_node('picamera', anonymous=False)
    rate = rospy.Rate(30) # 10Hz
    while not rospy.is_shutdown():
        rate.sleep()
        piCamera.run()




if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

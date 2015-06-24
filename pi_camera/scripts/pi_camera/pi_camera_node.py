#!/usr/bin/python

import json
import rospy
import sys
from PiCameraRosWrapper import PiCameraRosWrapper


def main():
    rospy.init_node('pi_camera_node', anonymous=False)
    piCamera = PiCameraRosWrapper()
    #rate = rospy.Rate(10) # 25Hz
    #piCamera.run_continuous()
    piCamera.run_continuous_threaded()
    #piCamera.run_sequential()



if __name__ == "__main__":
    #try:
    main()
    #except:
     # e = sys.exc_info()[0]
      #rospy.logfatal(e)
      #sys.exit(1)

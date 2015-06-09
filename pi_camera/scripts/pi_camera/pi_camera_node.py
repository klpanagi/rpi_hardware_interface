#!/usr/bin/python

import json
import rospy
from PiCameraRosWrapper import PiCameraRosWrapper


def main():
    rospy.init_node('pi_camera_node', anonymous=False)
    piCamera = PiCameraRosWrapper()
    #rate = rospy.Rate(10) # 25Hz
    piCamera.run_continuous()



if __name__ == "__main__":
    try:
        main()
    except:
      sys.exit(1)

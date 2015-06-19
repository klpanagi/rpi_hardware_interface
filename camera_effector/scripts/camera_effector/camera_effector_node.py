#!/usr/bin/python

import rospy
from time import sleep
from CameraEffector import CameraEffector

def main():
    rospy.init_node('camera_effector_node', anonymous=False)
    #rate = rospy.Rate(10) # 25Hz
    effector = CameraEffector()

    for i in range(-60,60,1):
        effector.move_servo_at_angle('pan', i)
        sleep(0.5)


if __name__ == "__main__":
    #try:
    main()
    #except rospy.ROSInterruptException:
        #pass

#!/usr/bin/python

import sys
import time
import json
from pi_camera import pi_camera_interface


def main():
    piCamera = pi_camera_interface.PiCameraInterface()
    #piCamera.startPreview()
    piCamera.capture_continuous()
    #piCamera.start_streaming()
    #piCamera.capture_sequence()



if __name__ == "__main__":
    try:
        main()
    except:
        sys.exit(0)


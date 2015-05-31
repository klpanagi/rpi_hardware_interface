#!/usr/bin/env python

from __future__ import with_statement # 2.5 only
import io
import time
import picamera
import picamera.array
import thread
from threading import Thread, Lock


## ---- RPI Camera Interfacing Class
class PiCameraInterface:

    ## TODO Load parameters from parameter server!!!
    ## TODO Dynamic Reconfigure

    ## Default constructor
    def __init__(self):
        self.res_width_ = 640
        self.res_height_ = 480
        self.framerate_ = 40
        self.image_format_ = 'rgb'
        self.use_video_port_ = True
        
        # Initiate camera module
        self.camera_ = picamera.PiCamera()
        #self.rgb_array_ = picamera.array.PiRGBArray(self.camera_)

        # Create in-memory stream object
        self.image_stream_ = io.BytesIO()
        self.frame_last_ = None
        self.mutex = Lock() # Create a mutex lock
    
        self.set_camera_params() # Set camera parameters

        # Create thread to handle with frame streaming
        try:
          #thread.start_new_thread(self.capture_sequence_toStream, ())
          self.streaming_thread_ = Thread( \
              target=self.capture_sequence_toStream, args=())
          self.streaming_thread_.start()
        except:
          print "Error: Unable to start Streaming Thread"
        #===========================================================

    ## Used by constructor to initiate camera parameters
    #   Do not invoke this Method!
    def set_camera_params(self):
        self.set_resolution(self.res_width_, self.res_height_)
        self.set_framerate(self.framerate_)


    ## Wrapper of start_preview() picamera functionality.
    def startPreview(self):
        self.camera_.start_preview()


    ## Wrapper of stop_preview() picamera functionality.
    def stopPreview(self):
        self.camera_.stop_preview()

    ## Sets camera resolution
    # @param width Image width (e.g 640)
    # @param height Image height (e.g. 480)
    def set_resolution(self, width, height):
        self.camera_.resolution = (width, height)

    ## Sets camera image capture framerate (fps)
    def set_framerate(self, framerate):
        self.camera_.framerate = framerate

    ## Close device port. Wrapping close() picamera functionality
    def closeDevice(self):
        self.camera_.close()
    
    ## Continues Streaming handler. 
    def stream(self):
        stream = io.BytesIO()
        while True:
          #start_time = time.time()
          # This returns the stream for the camera to capture to
          self.mutex.acquire(True)
          try:
            yield stream
            self.image_stream_ = stream
            # Extract frame raw bytes into the frame_current variable
            frame_current  = stream.getvalue()
            print 'Frame size: [\033[1;32m%s\033[0m]' % \
              len(frame_current)

            # Validate the frame then store the values.
            if len(frame_current) == self.res_height_ * self.res_width_ * 3:
              self.frame_last_ = frame_current
          finally:
            self.mutex.release()

          stream.truncate(0)
          stream.seek(0)
          # Finally, reset the stream for the next capture
          #stop_time = time.time()
          #exec_time = stop_time - start_time
          #print "CaptureImage Exec time: %s"%exec_time


    ## Returns last captured image frame from in-memory stream
    def get_image_from_stream(self):
        stream_frame = io.BytesIO()
        validFrame = False

        ## Loop until received a valid image frame
        while validFrame == False:
          self.mutex.acquire(True) # Lock block to access shared frame_last var
          try:
            rgb_frame = self.frame_last_
            # Validate last frame
            if rgb_frame != None :
              if len(rgb_frame) == self.res_height_ * self.res_width_ * 3:
                validFrame = True
                print 'ReturnStream size: [\033[1;32m%s\033[0m]' % \
                  len(rgb_frame)
              else:
                print 'ReturnStream size: [\033[1;31m%s\033[0m]' % \
                  len(rgb_frame)
          finally:
            self.mutex.release() # Release lock resources

        return rgb_frame

    ## Starts image frame streaming. Streaming is handled by a Thread
    def capture_sequence_toStream(self):
        self.camera_.capture_sequence(self.stream(), self.image_format_, self.use_video_port_)
        #print len(self.image_stream_.getvalue())


    # Captures and stores in a file!!
    def capture_image_file(self, name, form):
        fileName = name
        self.camera_.capture(fileName, form, True)
    ## Default destructor
    def __del__(self):
        self.camera_.close()
        del self.camera_
#------------------------------------------------------------------


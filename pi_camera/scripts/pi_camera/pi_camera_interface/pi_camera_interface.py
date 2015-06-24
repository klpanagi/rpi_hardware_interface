#!/usr/bin/env python

from __future__ import with_statement # 2.5 only
import io
import time
import picamera
import picamera.array
import thread
import sys
from threading import Thread, Lock
import timeit


## ---- RPI Camera Interfacing Class
class PiCameraInterface:

    ## TODO Load parameters from parameter server!!!
    ## TODO Dynamic Reconfigure

    #=======================================================================
    ## Default constructor
    def __init__(self):
        self.res_width_ = 640
        self.res_height_ = 480
        self.framerate_ = 30
        self.image_format_ = 'rgb'
        self.use_video_port_ = True
        
        # Initiate camera module
        self.camera_ = picamera.PiCamera()
        #self.rgb_array_ = picamera.array.PiRGBArray(self.camera_)

        # Create in-memory stream object
        self.image_stream_ = io.BytesIO()
        #stream_frame = io.BytesIO()
        self.frame_last_ = None
        self.mutex = Lock() # Create a mutex lock
        self.streaming_thread_ = None 
        self.set_camera_params() # Set camera parameters
        print "[PiCamera]: Waiting for camera to warmup for 2 seconds"
        #for i in range(0,2, -1):
            #print "[PiCamera]: %s" %i
            #time.sleep(1)
    #=======================================================================

    def start_streaming(self):
        # Create thread to handle with frame streaming
        try:
          #thread.start_new_thread(self.capture_sequence_toStream, ())
          self.streaming_thread_ = Thread( \
              target=self.capture_sequence_threaded, args=())
          self.streaming_thread_.start()
        except:
          print "Error: Unable to start Streaming Thread"
          sys.exit(1)
        finally:
            print "Streaming thread initialized succesfully"



    #=======================================================================
    ## Used by constructor to initiate camera parameters
    #   Do not invoke this Method!
    def set_camera_params(self):
        self.set_resolution(self.res_width_, self.res_height_)
        self.set_framerate(self.framerate_)
    #=======================================================================


    #=======================================================================
    ## Wrapper of start_preview() picamera functionality.
    def startPreview(self):
        self.camera_.start_preview()
    #=======================================================================


    #=======================================================================
    ## Wrapper of stop_preview() picamera functionality.
    def stopPreview(self):
        self.camera_.stop_preview()
    #=======================================================================


    #=======================================================================
    ## Sets camera resolution
    # @param width Image width (e.g 640)
    # @param height Image height (e.g. 480)
    def set_resolution(self, width, height):
        self.camera_.resolution = (width, height)
    #=======================================================================


    #=======================================================================
    ## Sets camera image capture framerate (fps)
    # @param framerate Camera capture rate in fps
    def set_framerate(self, framerate):
        self.camera_.framerate = framerate
    #=======================================================================


    #=======================================================================
    ## Close device port. 
    #   @brief Wrapping close() picamera functionality
    def closeDevice(self):
        self.camera_.close()
    #=======================================================================
    

    #=======================================================================
    ## Continues Streaming handler. 
    def stream(self):
        stream = io.BytesIO()
        while True:
          start_time = time.time()
          # This returns the stream for the camera to capture to
          self.mutex.acquire(True)
          try:
            yield stream
            self.image_stream_ = stream
            # Extract frame raw bytes into the frame_current variable
            frame_current  = stream.getvalue()
            #print 'Frame size: [\033[1;32m%s\033[0m]' % \
              #len(frame_current)

            # Validate the frame then store the values.
            if len(frame_current) == self.res_height_ * self.res_width_ * 3:
              self.frame_last_ = frame_current
          finally:
            self.mutex.release()

          # Finally, reset the stream for the next capture
          stream.truncate(0)
          stream.seek(0)
          stop_time = time.time()
          exec_time = stop_time - start_time
          print "CaptureImage Exec time: %s"%exec_time
    #=======================================================================


    #=======================================================================
    ## Returns last captured image frame from in-memory stream
    def get_image_from_stream(self):
        validFrame = False

        ## Loop until received a valid image frame
        while validFrame == False:
          # Lock block to access shared frame_last var
          self.mutex.acquire(True) 
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
                  #pass
          finally:
            self.mutex.release() # Release lock resources

        return rgb_frame
    #=======================================================================


    #=======================================================================
    ## Starts image frame streaming. Streaming is handled by a Thread
    def capture_sequence_threaded(self):
        self.camera_.capture_sequence(self.stream(), self.image_format_,\
                self.use_video_port_)
        #print len(self.image_stream_.getvalue())
    #=======================================================================

    ## Minimum capture elapsed time rated at 0.06 seconds average
    def capture_continuous(self):
        rawCapture = io.BytesIO()
        startT = timeit.default_timer()
        print "Capturing Continuous mode started"
        for frame in self.camera_.capture_continuous(rawCapture, \
                self.image_format_, self.use_video_port_):
            endT = timeit.default_timer()
            elapsedT = endT - startT
            startT = endT
            rawCapture.truncate(0)
            rawCapture.seek(0)
            print "Elapsed Time: [%s]" % elapsedT

    ## Minimum capture elapsed time rated at 0.06 seconds average
    def capture_sequence(self):
        rawCapture = io.BytesIO()
        startT = timeit.default_timer()
        print "Capturing sequential mode started"
        for frame in self.camera_.capture_sequence(rawCapture, \
                self.image_format_, self.use_video_port_):
            endT = timeit.default_timer()
            elapsedT = endT - startT
            startT = endT
            rawCapture.truncate(0)
            rawCapture.seek(0)
            print "Elapsed Time: [%s]" % elapsedT

    #=======================================================================
    ## Captures an image and store the data into a related file
    def capture_image_file(self, name, form):
        fileName = name
        self.camera_.capture(fileName, form, True)
    #=======================================================================


    #=======================================================================    
    ## Default destructor
    def __del__(self):
        self.camera_.close() # Safely terminate picamera interfaces
        del self.camera_
    #=======================================================================

#------------------------------------------------------------------------------


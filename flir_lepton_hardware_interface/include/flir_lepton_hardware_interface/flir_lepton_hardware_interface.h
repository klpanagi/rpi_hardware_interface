#ifndef FLIR_LEPTON_HARDWARE_INTERFACE_FLIR_LEPTON_HARDWARE_INTERFACE_H
#define FLIR_LEPTON_HARDWARE_INTERFACE_FLIR_LEPTON_HARDWARE_INTERFACE_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <cstring>
#include <limits.h>

#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"

namespace flir_lepton_hardware_interface
{
  void save_pgm_file(int maxval, int minval,
      float scale, const std::vector<uint16_t>& lepton_image);

  class FlirLeptonHardwareInterface
  {
    private:
      struct FlirSpi
      {
        uint8_t mode;
        uint8_t bits;
        uint32_t speed;
        uint16_t delay;
        uint16_t packet_size;
        uint16_t packets_per_frame;
        uint16_t packet_size_uint16;
        uint16_t frame_size_uint16;

        void configFlirSpi(const ros::NodeHandle& nh);
        uint8_t* makeFrameBuffer(void);
      };


    public:
      /*!
      * @brief Default constructor
      */
      FlirLeptonHardwareInterface(const std::string& ns);


      /*!
      * @brief Default Destructor
      */
      virtual ~FlirLeptonHardwareInterface();

      /*!
      * @brief Reads a thermal scene frame and publishes the image
      *  on the relevant topic
      */
      void run(void);

      /*!
      * @brief Converts a signal value to an actual image value by using
      *  a linear interpolation method.
      */
      uint8_t signalToImageValue(uint16_t signalValue, uint16_t minVal,
        uint16_t maxVal);


    private:
      /*!
      * @brief Opens SPI device port for communication with flir lepton camera
      */
      void openDevice(void);


      /*!
      * @brief Closes the SPI device communication port
      */
      void closeDevice(void);

      /*!
      * @brief Reads a frame from flir lepton thermal camera
      */
      void readFrame(uint8_t** frame_buffer);


      /*!
      * @brief Exports thermal signal values from an obtained VoSPI frame
      */
      void processFrame(
        uint8_t* frame_buffer, std::vector<uint16_t>* thermal_signals
        uint16_t* minValue, uint16_t* maxValue);

      /*!
      * @brief Currently unavailable
      * @TODO -- implement it!!!!!
      */
      void createMsg(
          const std::vector<uint16_t>& thermal_signals, sensor_msgs::Image* thermalImage
          uint16_t minValue, uint16_t maxValue);


    private:
      std::string flir_image_topic_;
      ros::Publisher flir_lepton_image_publisher_;
      ros::NodeHandle nh_;

      std::string device_;
      int spiDevice_;
      int statusValue_;
      FlirSpi flirSpi_;

      uint8_t* frame_buffer_;
      std::vector<uint16_t> thermal_signals_;

      ros::Time now_;
      std::string frame_id_;
      uint16_t imageHeight_;
      uint16_t imageWidth_;
  };
}  // namespace flir_lepton_hardware_interface

#endif  // FLIR_LEPTON_HARDWARE_INTERFACE_FLIR_LEPTON_HARDWARE_INTERFACE_H

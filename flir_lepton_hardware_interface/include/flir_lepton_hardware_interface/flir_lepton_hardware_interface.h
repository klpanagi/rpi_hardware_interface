#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <limits.h>
#include <cstring>
#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"


struct FlirSpi
{
  uint8_t mode;
  uint8_t bits;
  uint32_t speed;
  uint16_t delay;
  uint16_t packet_size;
  uint16_t packets_per_frame;
  uint16_t frame_size_uint16;
  uint16_t packet_size_uint16;
};


struct ThermalImage
{
  uint8_t width;
  uint8_t height;

};


class FlirLeptonHardwareInterface
{
  private:
    ros::Publisher flir_lepton_image_publisher_;
    ros::NodeHandle nh_;
    std::string device_;
    int spiDevice_;
    int statusValue_;
    uint8_t mode_;
    uint8_t bits_;
    uint32_t speed_;
    uint16_t delay_;
    uint16_t packet_size_;
    uint16_t packets_per_frame_;
    uint16_t frame_size_uint16_;
    uint16_t packet_size_uint16_;
    uint8_t* frame_buffer_;
    std::vector<uint16_t> thermal_signals_;
    ThermalImage imageT_;

  public:
    /*!
     * @brief Default constructor
     */
    FlirLeptonHardwareInterface(void);


    /*!
     * @brief Default Destructor
     */
    ~FlirLeptonHardwareInterface();


    /*!
     * @brief Reads a frame from flir lepton thermal camera
     */
    void readFrame(void);


    /*!
     * @brief Exports thermal signal values from an obtained VoSPI frame
     */
    void processFrame(void);


    /*!
     * @brief Reads a thermal scene frame and publishes the image
     *  on the relevant topic
     */
    void run(void);


    /*!
     * @brief Currently unavailable
     * @TODO -- implement it!!!!!
     */
    void createMsg();


    /*!
     * @brief Opens SPI device port for communication with flir lepton camera
     */
    void openDevice(void);


    /*!
     * @brief Closes the SPI device communication port
     */
    void closeDevice(void);


    /*!
     * @brief Converts a signal value to an actual image value by using
     *  a linear interpolation method.
     */
    static uint16_t signalToImageValue(uint16_t signalValue, uint16_t minVal,
      uint16_t maxVal);
};

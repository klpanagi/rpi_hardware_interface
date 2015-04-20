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
#include "sensor_msgs/Image.h"


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
    FlirLeptonHardwareInterface(void);


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
    static uint16_t signalToImageValue(uint16_t signalValue, uint16_t minVal,
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
    void readFrame(void);


    /*!
     * @brief Exports thermal signal values from an obtained VoSPI frame
     */
    void processFrame(void);

    /*!
     * @brief Currently unavailable
     * @TODO -- implement it!!!!!
     */
    void createMsg();


  private:
    ros::Publisher flir_lepton_image_publisher_;
    ros::NodeHandle nh_;
    std::string device_;
    int spiDevice_;
    int statusValue_;
    FlirSpi flirSpi_;
    uint8_t* frame_buffer_;
    std::vector<uint16_t> thermal_signals_;
    sensor_msgs::Image thermalImage_;
};

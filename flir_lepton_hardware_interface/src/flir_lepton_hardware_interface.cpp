#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <vector>

#include "sensor_msgs/Image.h"

#include "flir_lepton_hardware_interface/flir_lepton_hardware_interface.h"

namespace flir_lepton_hardware_interface
{

  FlirLeptonHardwareInterface::FlirLeptonHardwareInterface(const std::string& ns):
    nh_(ns),
    device_("/dev/spidev0.0")
  {
    int param;
    flirSpi_.configFlirSpi(nh_);
    frame_buffer_ = flirSpi_.makeFrameBuffer();
    nh_.param<std::string>("flir_urdf/camera_optical_frame", frame_id_, "/flir_optical_frame");
    nh_.param<int32_t>("thermal_image/height", param, 60);
    imageHeight_ = param;
    nh_.param<int32_t>("thermal_image/width", param, 80);
    imageWidth_ = param;
    openDevice();
    nh_.param<std::string>("published_topics/flir_image_topic", flir_image_topic_, "/flir_raspberry/image");
    flir_lepton_image_publisher_ = nh_.advertise<sensor_msgs::Image>(flir_image_topic_, 10);
  }


  FlirLeptonHardwareInterface::~FlirLeptonHardwareInterface()
  {
    closeDevice();
    delete[] frame_buffer_;
  }


  void FlirLeptonHardwareInterface::FlirSpi::configFlirSpi(const ros::NodeHandle& nh)
  {
    int param;
    mode = SPI_MODE_3;
    nh.param<int32_t>("flir_spi/bits", param, 8);
    bits = param;
    nh.param<int32_t>("flir_spi/speed", param, 24000000);
    speed = param;
    nh.param<int32_t>("flir_spi/delay", param, 0);
    delay = param;
    nh.param<int32_t>("flir_spi/packet_size", param, 164);
    packet_size = param;
    nh.param<int32_t>("flir_spi/packets_per_frame", param, 60);
    packets_per_frame = param;
    packet_size_uint16 = packet_size / 2;
    frame_size_uint16 = packet_size_uint16 * packets_per_frame;
  }


  uint8_t* FlirLeptonHardwareInterface::FlirSpi::makeFrameBuffer(void)
  {
    return new uint8_t[packet_size * packets_per_frame];
  }


  void FlirLeptonHardwareInterface::run(void)
  {
    readFrame(&frame_buffer_);
    now_ = ros::Time::now();
    thermal_signals_.clear();
    uint16_t minValue, maxValue;
    processFrame(frame_buffer_, &thermal_signals_, &minValue, &maxValue);
    sensor_msgs::Image thermalImage;
    createMsg(thermal_signals_, &thermalImage, minValue, maxValue);
    flir_lepton_image_publisher_.publish(thermalImage);
  }


  void FlirLeptonHardwareInterface::readFrame(uint8_t** frame_buffer)
  {
    int packet_number = -1;
    int resets = 0;

    for (uint16_t i = 0; i < flirSpi_.packets_per_frame; i++)
    {
      // flir sends discard packets that we need to resolve
      read(spiDevice_, &(*frame_buffer)[flirSpi_.packet_size * i],
        sizeof(uint8_t) * flirSpi_.packet_size);
      packet_number = (*frame_buffer)[i * flirSpi_.packet_size + 1];
      if (packet_number != i)
      {
        // if it is a drop packet, reset i
        i = -1;
        resets += 1;
        // sleep for 1ms
        ros::Duration(0.001).sleep();

        if (resets == 750) //Reach 750 sometimes
        {
          ROS_ERROR("[Flir-Lepton]: Error --> resets numbered at [%d]", resets);
          closeDevice();
          ros::Duration(1.0).sleep();
          openDevice();
          resets = 0;
        }
      }
    }
    //ROS_INFO("[Flir-Lepton]: Succesfully read of a single frame, resets=[%d]",
      //resets);
  }


  void FlirLeptonHardwareInterface::processFrame(
      uint8_t* frame_buffer, std::vector<uint16_t>* thermal_signals,
      uint16_t* minValue, uint16_t* maxValue)
  {
    int row, column;
    uint16_t value;
    *minValue = -1;
    *maxValue = 0;
    uint16_t* frame_buffer_16 = (uint16_t*) frame_buffer;
    uint16_t temp;
    uint16_t diff;
    float scale;
    std::vector<int> v;

    for (int i = 0; i < flirSpi_.frame_size_uint16; i++)
    {
      //Discard the first 4 bytes. it is the header.
      if (i % flirSpi_.packet_size_uint16 < 2) continue;

      temp = frame_buffer[i*2];
      frame_buffer[i*2] = frame_buffer[i*2+1];
      frame_buffer[i*2+1] = temp;
      value = frame_buffer_16[i];
      thermal_signals->push_back(value);
      if (value > *maxValue) *maxValue = value;
      if (value < *minValue) *minValue = value;
    }
  }


  /**
   * @details Thermal_signals is in column major as this is the way that the
   * packages are sent from the spi interface. Right now thermalImage's data is
   * being filled in a column major way.
   */
  void FlirLeptonHardwareInterface::createMsg(
      const std::vector<uint16_t>& thermal_signals, sensor_msgs::Image* thermalImage,
      uint16_t minValue, uint16_t maxValue)
  {
    thermalImage->header.stamp = now_;
    thermalImage->header.frame_id = frame_id_;

    thermalImage->height = imageHeight_;
    thermalImage->width = imageWidth_;

    thermalImage->encoding = "mono8";
    thermalImage->is_bigendian = 0;
    thermalImage->step = imageWidth_ * sizeof(uint8_t);

    for (int i = 0; i < imageWidth_; i++) {
      for (int j = 0; j < imageHeight_; j++) {
        uint8_t value = signalToImageValue(thermal_signals.at(i * imageHeight_ + j),
            minValue, maxValue);
        thermalImage->data.push_back(value);
      }
    }
  }


  uint8_t FlirLeptonHardwareInterface::signalToImageValue(uint16_t signal,
    uint16_t minVal, uint16_t maxVal)
  {
    uint16_t imageValue;
    uint16_t diff;
    float scale;
    diff = maxVal - minVal;
    scale = (float)255/diff;
    imageValue = (float)(signal - minVal)*scale;
    return (uint8_t) imageValue;
  }


  void FlirLeptonHardwareInterface::openDevice(void)
  {
    spiDevice_ = open(device_.c_str(), O_RDWR);
    if (spiDevice_ < 0)
    {
      ROS_FATAL("[Flir-Lepton]: Can't open SPI device");
      exit(1);
    }

    statusValue_ = ioctl(spiDevice_, SPI_IOC_WR_MODE, &flirSpi_.mode);
    if (statusValue_ < 0)
    {
      ROS_FATAL("[Flir-Lepton]: Can't set SPI-mode (WR)...ioctl failed");
      exit(1);
    }

    statusValue_ = ioctl(spiDevice_, SPI_IOC_RD_MODE, &flirSpi_.mode);
    if (statusValue_ < 0)
    {
      ROS_FATAL("[Flir-Lepton]: Can't set SPI-mode (RD)...ioctl failed");
      exit(1);
    }

    statusValue_ = ioctl(spiDevice_, SPI_IOC_WR_BITS_PER_WORD, &flirSpi_.bits);
    if (statusValue_ < 0)
    {
      ROS_FATAL("[Flir-Lepton]: Can't set SPI bitsperWord (WD)...ioctl failed");
      exit(1);
    }

    statusValue_ = ioctl(spiDevice_, SPI_IOC_RD_BITS_PER_WORD, &flirSpi_.bits);
    if (statusValue_ < 0)
    {
      ROS_FATAL("[Flir-Lepton]: Can't set SPI bitsperWord (RD)...ioctl failed");
      exit(1);
    }

    statusValue_ = ioctl(spiDevice_, SPI_IOC_WR_MAX_SPEED_HZ, &flirSpi_.speed);
    if (statusValue_ < 0)
    {
      ROS_FATAL("[Flir-Lepton]: Can't set SPI speed (WD)...ioctl failed");
      exit(1);
    }

    statusValue_ = ioctl(spiDevice_, SPI_IOC_RD_MAX_SPEED_HZ, &flirSpi_.speed);
    if (statusValue_ < 0)
    {
      ROS_FATAL("[Flir-Lepton]: Can't set SPI speed (RD)...ioctl failed");
      exit(1);
    }
    ROS_WARN("[Flir-Lepton]: Opened SPI Port");
  }


  void FlirLeptonHardwareInterface::closeDevice(void)
  {
    statusValue_ = close(spiDevice_);
    if (statusValue_ < 0)
    {
      ROS_FATAL("[Flir-Lepton]: Could not close SPI device");
      exit(1);
    }
    ROS_WARN("[Flir-Lepton]: Closed SPI Port");
  }

}  // namespace flir_lepton_hardware_interface

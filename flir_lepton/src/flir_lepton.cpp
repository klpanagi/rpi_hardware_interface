/*********************************************************************
 * *
 * * Software License Agreement (BSD License)
 * *
 * *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
 * *  All rights reserved.
 * *
 * *  Redistribution and use in source and binary forms, with or without
 * *  modification, are permitted provided that the following conditions
 * *  are met:
 * *
 * *   * Redistributions of source code must retain the above copyright
 * *     notice, this list of conditions and the following disclaimer.
 * *   * Redistributions in binary form must reproduce the above
 * *     copyright notice, this list of conditions and the following
 * *     disclaimer in the documentation and/or other materials provided
 * *     with the distribution.
 * *   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
 * *     contributors may be used to endorse or promote products derived
 * *     from this software without specific prior written permission.
 * *
 * *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * *  POSSIBILITY OF SUCH DAMAGE.
 * *
 * * Author: Konstantinos Panayiotou, Aggelos Triantafillidis,
 * *  Tsirigotis Christos
 * * Maintainer: Konstantinos Panayiotou
 * * Email: klpanagi@gmail.com
 * *********************************************************************/

#include "flir_lepton/flir_lepton.h"

#define MIN_VALUE 7800
#define MAX_VALUE 8600

namespace flir_lepton
{

  FlirLeptonHardwareInterface::FlirLeptonHardwareInterface(const std::string& ns):
    nh_(ns),
    device_("/dev/spidev0.0")
  {
    int param;
    // Fill the data map
    dataMap_ = fillCalibrationMap();
    flirSpi_.configFlirSpi(nh_);
    frame_buffer_ = flirSpi_.makeFrameBuffer();

    /* ----------- Load Parameters ------------ */
    nh_.param<std::string>("flir_urdf/camera_optical_frame", frame_id_,
      "/flir_optical_frame");
    nh_.param<int32_t>("thermal_image/height", param, 60);
    imageHeight_ = param;
    nh_.param<int32_t>("thermal_image/width", param, 80);
    imageWidth_ = param;
    nh_.param<std::string>("published_topics/flir_image_topic", image_topic_,
      "/rpi2/thermal/image");
    nh_.param<std::string>("published_topics/flir_fused_topic",
      fusedMsg_topic_, "/rpi2/thermal/fused_msg");
    /* ----------------------------------------- */
    MAX_RESETS_ERROR = 750;
    MAX_RESTART_ATTEMPS_EXIT = 5;

    openDevice();
    fusedMsg_publisher_ = nh_.advertise<distrib_msgs::flirLeptonMsg>(
      fusedMsg_topic_, 1);
    image_publisher_ = nh_.advertise<sensor_msgs::Image>(image_topic_, 1);
  }


  FlirLeptonHardwareInterface::~FlirLeptonHardwareInterface()
  {
    closeDevice();
    delete[] frame_buffer_;
  }


  void FlirLeptonHardwareInterface::FlirSpi::configFlirSpi(
    const ros::NodeHandle& nh)
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

    // Sensor_msgs/Image
    sensor_msgs::Image thermalImage;

    // Custom message
    distrib_msgs::flirLeptonMsg fusedMsg;

    // Create the Image message
    fill_ImageMsg(thermal_signals_, &thermalImage, minValue, maxValue);

    // Create the custom message
    fill_fusedMsg(thermal_signals_, &fusedMsg, minValue, maxValue);

    image_publisher_.publish(thermalImage);

    //image_publisher_.publish(flirMsg);    

    fusedMsg_publisher_.publish(fusedMsg);    
  }


  void FlirLeptonHardwareInterface::readFrame(uint8_t** frame_buffer)
  {
    int packet_number = -1;
    int resets = 0;
    int restarts = 0;

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

        // If resets reach this value, we assume an error on communication with
        // flir-lepton sensor. Perform communication restart
        if (resets == MAX_RESETS_ERROR) //Reach 750 sometimes
        {
          restarts ++;
          ROS_ERROR("[Flir-Lepton]: Error --> resets numbered at [%d]", resets);
          closeDevice();
          ros::Duration(1.0).sleep();
          openDevice();
          resets = 0;
        }

        if (restarts > MAX_RESTART_ATTEMPS_EXIT)
        {
          // If true we assume an exit status.. Kill process and exit
          ROS_FATAL("[Flir-Lepton]: Cannot communicate with sensor. Exiting...");
          ros::shutdown();
          exit(1);
        }
      }
    }
    restarts = 0;
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
  void FlirLeptonHardwareInterface::fill_ImageMsg(
      const std::vector<uint16_t>& thermal_signals, 
        sensor_msgs::Image* thermalImage, uint16_t minValue, uint16_t maxValue)
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

  void FlirLeptonHardwareInterface::fill_fusedMsg(
    const std::vector<uint16_t>& thermal_signals, 
    distrib_msgs::flirLeptonMsg* flirMsg, 
    uint16_t minValue, uint16_t maxValue)
  {
    flirMsg->header.stamp = now_;
    flirMsg->header.frame_id = frame_id_;

    flirMsg->thermalImage.header.stamp = now_;
    flirMsg->thermalImage.header.frame_id = frame_id_;
    flirMsg->thermalImage.height = imageHeight_;
    flirMsg->thermalImage.width = imageWidth_;
    flirMsg->thermalImage.encoding = "mono8";
    flirMsg->thermalImage.is_bigendian = 0;
    flirMsg->thermalImage.step = imageWidth_ * sizeof(uint8_t);

    // Thermal sensor_msgs/Image 
    for (int i = 0; i < imageHeight_; i++) {
      for (int j = 0; j < imageWidth_; j++) {
        uint8_t value = signalToImageValue(thermal_signals.at(i * imageWidth_ + j),
          minValue, maxValue);
        flirMsg->thermalImage.data.push_back(value);
      }
    }
  
    // Vector containing the temperatures in image after calibration and vector
    // with signal raw values 
    for (int i = 0; i < imageHeight_; i++) {
      for (int j = 0; j < imageWidth_; j++) {
        flirMsg->rawValues.data.push_back(thermal_signals.at(i * imageWidth_ + j));

        float value = signalToTemperature(thermal_signals.at(i * imageWidth_ + j));
        flirMsg->temperatures.data.push_back(value);
       
      }
    }
    flirMsg->temperatures.layout.dim.push_back(std_msgs::MultiArrayDimension());
    flirMsg->temperatures.layout.dim[0].size = 60;

    flirMsg->temperatures.layout.dim.push_back(std_msgs::MultiArrayDimension());
    flirMsg->temperatures.layout.dim[1].size = 80;
  }

  float FlirLeptonHardwareInterface::signalToTemperature(uint16_t signalValue)
  {
    // The input signalValue is the keyword of the map with temperatures
    std::map<uint16_t, float>::iterator search = dataMap_.find(signalValue);
    float give_temp;

    if(search != dataMap_.end())
    {
      // Pass the value from the map
      give_temp = search->second;
    }
    else
    {
      give_temp = 0;
    }
    
    return give_temp;    
  }


  /*!
   * @brief Loads the temperature-raw_signal_values relation from the dataset 
   *  file.
   *  @return A map which containes temperature-signal indexes
   */
  std::map<uint16_t, float> FlirLeptonHardwareInterface::fillCalibrationMap(void)
  {
    char* dataset_uri = new char[128];
    std::string param;

    /* ---< Load dataset file from parameter server >--- */
    //nh_.getParam("dataset/spline_interpolated_data", param);
    nh_.param<std::string>("dataset/spline_interpolated_data", param, 
      "/home/pandora/pandora_ws/src/rpi_hardware_interface/data" \
      "/flir_lepton/dataset_spline_interp.pandora");

    strcpy(dataset_uri, param.c_str());
    /* ---< Open a file input stream to read the dataset >--- */
    std::ifstream file(dataset_uri);
    std::string line;
    
    std::map<uint16_t, float> dataMap;

    // Value to be stored in map and its name in the file that we read
    float value = 0;
    uint16_t keyword = 0;
    std::string value_s;
    std::string keyword_s;
    std::istringstream ss;

    // If counter even -> read keyword of map from file
    // If counter odd -> read value of map from file
    int counter =2;
    
    if(file.is_open())
    {
      while(getline(file, line))
      {
        if(counter % 2 == 0)
        { 
          // Read the keyword of map and convert it to uint16_t
          keyword_s = line;
          
          std::istringstream ss(keyword_s);
          ss >> keyword;

         if(ss.fail())
         {
           ROS_ERROR("Failed to read thermal-signal dataset");
           exit(1);
         }
        }
        else
        {
          value_s = line;

          // Convert string to float for map
          std::istringstream ss(value_s);
          ss >> value;

          if(ss.fail())
          {
            ROS_ERROR("Failed to read thermal-signal dataset");
            exit(1);
          }
          // Fill the map with the next pair
          dataMap[keyword] = value;
        }
        counter++;
      }
      file.close();
    }
    else
    {
      ROS_ERROR("Failed to open file");
      exit(1);
    }
    // Check if map size is ok
    //ROS_INFO_STREAM("MAP SIZE =" << dataMap.size());
    return dataMap;
  }


  /*!
   * @brief Converts signal values to raw_image values.
   * @param signal signal value
   * @param minVal Minimun signal value captured on a thermal image frame.
   * @param maxVal Maximum signal value captured on a thermal image frame.
   * @return raw_image value.
   */
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


  /*!
   * @brief Opens communication port to flirlepton sensor
   * @return Void.
   */
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

  /*!
   * @brief Closes communication port to flir-lepton sensor
   * @return Void
   */
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

}  // namespace flir_lepton

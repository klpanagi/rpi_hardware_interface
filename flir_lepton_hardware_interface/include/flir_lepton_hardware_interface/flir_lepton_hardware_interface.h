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
    uint16_t packetSize_;
    uint16_t packets_per_frame_;
    uint16_t frame_size_uint16_;
    uint16_t packet_size_uint16_;
    uint8_t* lepton_frame_;
    unsigned int lepton_image_[80][80];

  public:
    FlirLeptonHardwareInterface(void);
    ~FlirLeptonHardwareInterface();
    void readFrame(void);
    void processFrame(void);
    void run(void);
    void createMsg();
    void openDevice(void);
    void closeDevice(void);
};

#include "flir_lepton_hardware_interface/flir_lepton_hardware_interface.h"


FlirLeptonHardwareInterface::FlirLeptonHardwareInterface(const std::string& ns):
  nh_(ns),
  device_("/dev/spidev0.0"),
{
  flirSpi_.configFlirSpi(nh_);
  frame_buffer_ = flirSpi_.makeFrameBuffer();
  flir_lepton_image_publisher_ = nh_.advertise<sensor_msgs::Image>("/flir_raspberry/image", 10);
  openDevice();
}


void FlirLeptonHardwareInterface::FlirSpi::configFlirSpi(const ros::NodeHandle& nh)
{
  mode = SPI_MODE_3
  nh.param("flir_spi/bits", bits, 8);
  nh.param("flir_spi/speed", speed, 24000000);
  nh.param("flir_spi/delay", delay, 0);
  nh.param("flir_spi/packet_size", packet_size, 164);
  nh.param("flir_spi/packets_per_frame", packets_per_frame, 60);
  packet_size_uint16 = packet_size / 2;
  frame_size_uint16 = packet_size_uint16 * packets_per_frame;
}


int8_t* FlirLeptonHardwareInterface::FlirSpi::makeFrameBuffer(void)
{
  return new uint8_t[packet_size * packets_per_frame];
}


void FlirLeptonHardwareInterface::run(void)
{
  readFrame();
  thermal_signals_.clear();
  processFrame();
}


void FlirLeptonHardwareInterface::readFrame(void)
{
  int packet_number = -1;
  int resets = 0;

  for(uint16_t i = 0; i < flirSpi_.packets_per_frame; i++)
  {
    // flir sends discard packets that we need to resolve
    read(spiDevice_, &frame_buffer_[packet_size_*i],
      sizeof(uint8_t)*packet_size_);
    packet_number = frame_buffer_[i*packet_size_+1];
    if(packet_number != i)
    {
      //if it is a drop packet, reset i
      i = -1;
      resets += 1;
      //sleep for 1ms
      ros::Duration(0.001).sleep();

      if(resets == 750)//Reach 750 sometimes
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

void save_pgm_file(int maxval, int minval, float scale, const std::vector<int>& lepton_image)
{
  int i;
  int j;

  FILE *f = fopen("/home/pandora/image.pgm", "w");
  if (f == NULL)
  {
    printf("Error opening file!\n");
    exit(1);
  }

  printf("maxval = %u\n",maxval);
  printf("minval = %u\n",minval);
  printf("scale = %f\n",scale);

  fprintf(f,"P2\n80 60\n%u\n",maxval-minval);
  for(i=0;i<lepton_image.size();i++)
  {
      //Discard the first 4 bytes. it is the header.
      //std::cout << lepton_image[i] << " ";
      fprintf(f,"%d ", (lepton_image.at(i) - minval));
  }
  fprintf(f,"\n\n");

  fclose(f);
}


void FlirLeptonHardwareInterface::processFrame(void)
{
  int row, column;
  uint16_t value;
  uint16_t minValue = -1;
  uint16_t maxValue = 0;
  uint16_t* frame_buffer =  (uint16_t *)frame_buffer_;
  uint16_t temp;
  uint16_t diff;
  float scale;
  std::vector<int> v;

  for(int i=0;i<frame_size_uint16_;i++)
  {
    //Discard the first 4 bytes. it is the header.
    if(i%packet_size_uint16_ < 2) continue;

    temp = frame_buffer_[i*2];
    frame_buffer_[i*2] = frame_buffer_[i*2+1];
    frame_buffer_[i*2+1] = temp;
    value = frame_buffer[i];
    //std::cout << value << " ";
    thermal_signals_.push_back(value);
    if(value > maxValue) maxValue = value;
    if(value < minValue) minValue = value;
  }

  std_msgs::UInt8MultiArray _image;
  _image.layout.dim.push_back(std_msgs::MultiArrayDimension());
  _image.layout.dim.push_back(std_msgs::MultiArrayDimension());
  _image.layout.dim[0].size = imageT_.width;
  _image.layout.dim[1].size = imageT_.height;

  for(int i = 0 ; i < imageT_.width ; i++)
    for(int j = 0 ; j < imageT_.height ; j++)
      _image.data.push_back( signalToImageValue(thermal_signals_.at(i*imageT_.height + j),
          minValue, maxValue) );
  flir_lepton_image_publisher_.publish(_image);
}


void FlirLeptonHardwareInterface::createMsg(void)
{

}


uint16_t FlirLeptonHardwareInterface::signalToImageValue(uint16_t signal,
  uint16_t minVal, uint16_t maxVal)
{
  uint16_t imageValue;
  uint16_t diff;
  float scale;
  diff = maxVal - minVal;
  scale = (float)255/diff;
  imageValue = (float)(signal - minVal)*scale;
  return imageValue;
}


void FlirLeptonHardwareInterface::openDevice(void)
{
  spiDevice_ = open(device_.c_str(), O_RDWR);
  if (spiDevice_ < 0)
  {
    ROS_FATAL("[Flir-Lepton]: Can't open SPI device");
    exit(1);
  }

  statusValue_ = ioctl(spiDevice_, SPI_IOC_WR_MODE, &mode_);
  if (statusValue_ < 0)
  {
    ROS_FATAL("[Flir-Lepton]: Can't set SPI-mode (WR)...ioctl failed");
    exit(1);
  }

  statusValue_ = ioctl(spiDevice_, SPI_IOC_RD_MODE, &mode_);
  if (statusValue_ < 0)
  {
    ROS_FATAL("[Flir-Lepton]: Can't set SPI-mode (RD)...ioctl failed");
    exit(1);
  }

  statusValue_ = ioctl(spiDevice_, SPI_IOC_WR_BITS_PER_WORD, &bits_);
  if (statusValue_ < 0)
  {
    ROS_FATAL("[Flir-Lepton]: Can't set SPI bitsperWord (WD)...ioctl failed");
    exit(1);
  }

  statusValue_ = ioctl(spiDevice_, SPI_IOC_RD_BITS_PER_WORD, &bits_);
  if (statusValue_ < 0)
  {
    ROS_FATAL("[Flir-Lepton]: Can't set SPI bitsperWord (RD)...ioctl failed");
    exit(1);
  }

  statusValue_ = ioctl(spiDevice_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_);
  if (statusValue_ < 0)
  {
    ROS_FATAL("[Flir-Lepton]: Can't set SPI speed (WD)...ioctl failed");
    exit(1);
  }

  statusValue_ = ioctl(spiDevice_, SPI_IOC_RD_MAX_SPEED_HZ, &speed_);
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
  if(statusValue_ < 0)
  {
    ROS_FATAL("[Flir-Lepton]: Could not close SPI device");
    exit(1);
  }
  ROS_WARN("[Flir-Lepton]: Closed SPI Port");
}


FlirLeptonHardwareInterface::~FlirLeptonHardwareInterface()
{
  closeDevice();
  delete frame_buffer_;
}

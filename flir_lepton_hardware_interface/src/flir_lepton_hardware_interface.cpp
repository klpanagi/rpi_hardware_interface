#include "flir_lepton_hardware_interface/flir_lepton_hardware_interface.h"

int count = 0;

FlirLeptonHardwareInterface::FlirLeptonHardwareInterface(void):
  device_("/dev/spidev0.0"),
  speed_(10000000),
  bits_(8),
  packetSize_(164),
  mode_(SPI_MODE_3),
  packets_per_frame_(60)
{
  packet_size_uint16_ = packetSize_/2;
  frame_size_uint16_ = (packetSize_/2)*packets_per_frame_;
  lepton_frame_ = new uint8_t[packetSize_*packets_per_frame_];
  openDevice();
  flir_lepton_image_publisher_ = nh_.advertise<std_msgs::UInt8MultiArray>("/flir_raspberry/image", 1000);
}


void FlirLeptonHardwareInterface::run(void)
{
  readFrame();
  processFrame();

}


void FlirLeptonHardwareInterface::readFrame(void)
{
  int packet_number = -1;
  int resets = 0;

  for(uint16_t i=0;i<packets_per_frame_;i++)
  {
    // flir sends discard packets that we need to resolve
    read(spiDevice_, &lepton_frame_[packetSize_*i], 
      sizeof(uint8_t)*packetSize_);
    packet_number = lepton_frame_[i*packetSize_+1];
    if(packet_number != i)
    {
      //if it is a drop packet, reset i
      i = -1;
      resets += 1;
      usleep(1000); //sleep for 1ms

      if(resets == 750)
      {
        closeDevice();
        usleep(750000);
        openDevice();
      }
    }
  }
  ROS_INFO("[Flir-Lepton]: Succesfully read of a single frame, resets=[%d]",
    resets);
}

void save_pgm_file(int maxval, int minval, float scale, std::vector<int>& lepton_image)
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
  uint32_t value;
  uint32_t minValue = 50000;
  uint32_t maxValue = 0;
  uint16_t* frame_buffer =  (uint16_t *)lepton_frame_;
  uint32_t temp;
  uint16_t diff;
  float scale;
  std::vector<int> v;

  for(int i=0;i<frame_size_uint16_;i++)
  {
    //Discard the first 4 bytes. it is the header.
    if(i%packet_size_uint16_ < 2)
    {
      //ROS_FATAL("asd");
      continue;
    }
    
    temp = lepton_frame_[i*2];
    lepton_frame_[i*2] = lepton_frame_[i*2+1];
    lepton_frame_[i*2+1] = temp;
    value = frame_buffer[i];
    //std::cout << value << " ";
    v.push_back(value);
    if(value > maxValue)
    {
      maxValue = value;
    }
    if(value < minValue) minValue = value;

  }
  diff = maxValue - minValue;
  scale = (float)255/diff;

  std_msgs::UInt8MultiArray _image;
  _image.layout.dim.push_back(std_msgs::MultiArrayDimension ());
  _image.layout.dim.push_back(std_msgs::MultiArrayDimension ());
  _image.layout.dim[0].size = 80;
  _image.layout.dim[1].size = 60;

  //_image.data.clear();
  //_image.data = std::vector<unsigned char>(frame_size_uint16_);

  for(int i = 0 ; i < 80 ; i++)
    for(int j = 0 ; j < 60 ; j++)
      _image.data.push_back( (float)(v.at(i*60 + j) - minValue)*scale );
  //for(int i=0;i<frame_size_uint16_;i++)
  //{
    //_image.data[(i/80)*60 + i%80] = 1; //( (float)(frame_buffer[i]-minValue)*scale ); 
    //_image.data[i] =1;
  //}
  flir_lepton_image_publisher_.publish(_image);
  //if(count == 0)
    //save_pgm_file(maxValue, minValue, scale, v);
  //count++;
}

void FlirLeptonHardwareInterface::createMsg(void)
{
  
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
}

void FlirLeptonHardwareInterface::closeDevice(void)
{
  statusValue_ = close(spiDevice_);
  if(statusValue_ < 0)
  {
    ROS_FATAL("[Flir-Lepton]: Could not close SPI device");
    exit(1);
  }
}

//void FlirLeptonHardwareInterface::createMsg(void)
//{
  //unsigned int i,j;
  //unsigned int maxval_ = 0, minval_ = -1;
  //for(i=0;i<60;i++)
  //{
    //for(j=0;j<80;j++)
    //{
      //if (lepton_image_[i][j] > maxval_) {
        //maxval_ = lepton_image_[i][j];
      //}
      //if (lepton_image_[i][j] < minval_) {
        //minval_ = lepton_image_[i][j];
      //}
    //}
  //}

  //std_msgs::UInt8MultiArray _image;

  //for(i=0;i<80;i++)
  //{
    //for(j=0;j<60;j++)
    //{
      //_image.data.push_back(((float)lepton_image_[j][i] - minval_) / (maxval_ - minval_) * 255) ;
    //}
  //}
  //flir_lepton_image_publisher_.publish(_image);
  //ROS_FATAL("Publishing image");
//}


//int FlirLeptonHardwareInterface::read(void)
//{
  //int ret;
  //int frame_number;
  //int i;
  //uint8_t tx[164];
  //spi_ioc_transfer tr;
  //for(unsigned int i = 0 ; i < 164 ; i++) tx[i] = 0;
  //tr.tx_buf = (unsigned long)tx;
  //tr.rx_buf = (unsigned long)lepton_frame_packet_;
  //tr.len = 164;
  //tr.delay_usecs = 0;
  //tr.speed_hz = 10000000;
  //tr.bits_per_word = 8; 
  
  //usleep(1000);
  //ROS_FATAL("Bitch");
  //ROS_INFO("spiDevice: [%d]", spiDevice_);
  //statusValue_ = ioctl(spiDevice_, SPI_IOC_MESSAGE(1), &tr);
  //if (statusValue_ < 1)
    //ROS_FATAL("Can't send spi message: Error[%d]", statusValue_);

  //if(((lepton_frame_packet_[0]&0xf) != 0x0f))
  //{
    //frame_number = lepton_frame_packet_[1];
    //ROS_INFO("Frame Number: [%d]", frame_number);

    //if(frame_number < 60 )
    //{
      //for(i=0;i<80;i++)
      //{
        //lepton_image_[frame_number][i] = (lepton_frame_packet_[2*i+4] << 8 | lepton_frame_packet_[2*i+5]);
      //}
    //}
  //}
  //return frame_number;
//}

FlirLeptonHardwareInterface::~FlirLeptonHardwareInterface()
{
  closeDevice();
}

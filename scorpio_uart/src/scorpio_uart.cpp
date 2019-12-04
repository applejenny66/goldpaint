#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <iomanip>
#include "scorpio_uart/scorpio_uart.h"


using namespace LibSerial ;    

//open the serial port
SerialHandle::SerialHandle()
{
}

SerialHandle::~SerialHandle()
{
    serial_port_.Close();
}

void SerialHandle::set_port(const char* const port_name)
{
  serial_port_.Open(port_name);
  if (!serial_port_.good()) 
    ROS_ERROR("Could not open serial port %s", port_name);
}

//set baud rate
void SerialHandle::set_baudRate(int baud_rate)
{
  if(baud_rate == 9600)
  {
    serial_port_.SetBaudRate(SerialStreamBuf::BAUD_9600);
    if (!serial_port_.good()) 
      ROS_ERROR("Error: Could not set the baud rate %d", baud_rate);
  }
  else if(baud_rate == 115200)
  {
    serial_port_.SetBaudRate(SerialStreamBuf::BAUD_115200);
    if (!serial_port_.good()) 
      ROS_ERROR("Error: Could not set the baud rate %d", baud_rate);
  }
  else
    ROS_ERROR("No this baud rate option. Please check scorpio_uart.cpp.");
}

//set the num of data bits
void SerialHandle::set_dataBits(int data_bits)
{
  if(data_bits == 8)
  {
    serial_port_.SetCharSize(SerialStreamBuf::CHAR_SIZE_8) ;
    if (!serial_port_.good()) 
      ROS_ERROR("Could not set data bits %d", data_bits);
  }
  else
    ROS_ERROR("Please set data bits 8");
}

//set stop bit
void SerialHandle::set_stopBit(int stop_bit)
{
  serial_port_.SetNumOfStopBits(stop_bit) ;
  if(stop_bit == 1)
  {
    if(!serial_port_.good()) 
      ROS_ERROR("Could not set stop bit %d", stop_bit);
  }
  else
    ROS_ERROR("Please set stop bit 1");

}

//set parity
void SerialHandle::set_parity(bool parity)
{
  //disable parity
  if(!parity)
  {
    serial_port_.SetParity( SerialStreamBuf::PARITY_NONE);
    if (!serial_port_.good()) 
      ROS_ERROR("Could not disable the parity.");
  }
  else
    ROS_ERROR("Please disable the parity.");
}

//set hardware flow control
void SerialHandle::set_hardwareFlowControl(bool control)
{
  //disable hardware flow control
  if(!control)
  {
    serial_port_.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
    if (!serial_port_.good()) 
      ROS_ERROR("Could not disable hardware flow control.");
  }
  else
    ROS_ERROR("Please disable hardware flow contorl.");
}

void SerialHandle::calcCRC_()
{
  // Init CRC
  uint8_t u8CRC = calcCRCByte_(obuff_[1], 0xFF);

  // Calculate following CRC
  int pkg_len = obuff_[1];
  for(int i = 2; i < pkg_len; i++)
  {
    u8CRC = calcCRCByte_(obuff_[i], u8CRC);
  }

  obuff_[pkg_len] = u8CRC;
}

bool SerialHandle::verifyCRC_()
{
  // Init CRC
  uint8_t u8CRC = calcCRCByte_((uint8_t)ibuff_[1], 0xFF);

  // Calculate following CRC
  int pkg_len = ibuff_[1];
  for(int i = 2; i < pkg_len; i++)
  {
    u8CRC = calcCRCByte_((uint8_t)ibuff_[i], u8CRC);
  }

  if(u8CRC == (uint8_t)ibuff_[pkg_len])
    return true;
  else
    return false;
}

uint8_t SerialHandle::calcCRCByte_(uint8_t u8Byte, uint8_t u8CRC)
{
  u8CRC = u8CRC ^ u8Byte;

  for(int i = 0; i < 8; i++)  
  {
    if(u8CRC & 0x01)
    {
      u8CRC = (u8CRC >> 1) ^ 0xD5;
    }
    else
    {
      u8CRC >>= 1;
    }
  }
  return u8CRC;
}

void SerialHandle::showPackage_(uint8_t buff[], int len)
{
  int dec_temp;
  for(int i = 0; i < len; i++)
  {
    dec_temp = buff[i];
    std::cout << std::hex << std::uppercase << dec_temp << " ";
  }
  std::cout << std::endl;
}

void SerialHandle::SDOS32dataWrapper_(int32_t data)
{
    obuff_[DATA_ADDR] = (uint8_t)data;
    obuff_[DATA_ADDR + 1] = (uint8_t)(data >> 8);
    obuff_[DATA_ADDR + 2] = (uint8_t)(data >> 16);
    obuff_[DATA_ADDR + 3] = (uint8_t)(data >> 24);
}

void SerialHandle::SDOdataWrapper_(int bytes, uint16_t data)
{
  if(bytes == 1)
  {
    obuff_[DATA_ADDR] = (uint8_t)data;
  }
  else if(bytes == 2)
  {
    obuff_[DATA_ADDR] = (uint8_t)data;
    obuff_[DATA_ADDR + 1] = (uint8_t)(data >> 8);
  }
  else if(bytes == 4)
  {
    obuff_[DATA_ADDR] = (uint8_t)data;
    obuff_[DATA_ADDR + 1] = (uint8_t)(data >> 8);
    obuff_[DATA_ADDR + 2] = (uint8_t)(data >> 16);
    obuff_[DATA_ADDR + 3] = (uint8_t)(data >> 24);
  }
  else
  {
    ROS_ERROR("No such data length.");
  }
}

void SerialHandle::SDOwriteWrapper_(int len, int node, uint16_t index, uint8_t sub_index)
{
  obuff_[0] = 0x53;
  obuff_[1] = (uint8_t)len;
  obuff_[2] = (uint8_t)node;
  obuff_[3] = 0x02;
  obuff_[4] = (uint8_t)index;
  obuff_[5] = (uint8_t)(index >> 8);
  obuff_[6] = sub_index;
  obuff_[len + 1] = 0x45;
}

void SerialHandle::SDOreadWrapper_(int node, uint16_t index, uint8_t sub_index)
{
  obuff_[0] = 0x53;
  obuff_[1] = (uint8_t)READ_CMD_LEN; 
  obuff_[2] = (uint8_t)node;
  obuff_[3] = 0x01;
  obuff_[4] = (uint8_t)index;
  obuff_[5] = (uint8_t)(index >> 8);
  obuff_[6] = sub_index;
  obuff_[8] = 0x45;
}

void SerialHandle::consumeResponse_()
{
  char next_byte;

  while(1)
  {
    serial_port_.get(next_byte);
    if(next_byte == 0x53)
    {
      //ibuff_[count++] = next_byte;
      break;
    }
  }
  while(1)
  {
    serial_port_.get(next_byte);
    //ibuff_[count++] = next_byte;
    if(next_byte == 0x45)
      break;
  }
}

void SerialHandle::switchOn(int node)
{
  // Package 1 (opmod3, profile velocity)
  SDOwriteWrapper_(8, node, 0x6060, 0x00);
  SDOdataWrapper_(1, 0x03);
  calcCRC_(); 
  serial_port_.write((char *)obuff_, 11);
  consumeResponse_();

  int pkg_len = 9;
  int data_len = 2;
  uint16_t index = 0x6040;
  uint8_t sub_index = 0x00;

  // Wrap basic data
  SDOwriteWrapper_(pkg_len, node, index, sub_index);

  // Package 2
  SDOdataWrapper_(data_len, 0x0E);
  calcCRC_(); 
  serial_port_.write((char *)obuff_, pkg_len + 2);
  consumeResponse_();

  // Package 3
  SDOdataWrapper_(data_len, 0x0F);
  calcCRC_(); 
  serial_port_.write((char *)obuff_, pkg_len + 2);
  consumeResponse_();
}

void SerialHandle::switchOff(int node)
{
  int pkg_len = 9;
  int data_len = 2;
  uint16_t index = 0x6040;
  uint8_t sub_index = 0x00;

  SDOwriteWrapper_(pkg_len, node, index, sub_index);
  SDOdataWrapper_(data_len, 0x07);
  calcCRC_(); 
  serial_port_.write((char *)obuff_, pkg_len + 2);
  consumeResponse_();
}

void SerialHandle::homing(int node)
{
  int pkg_len;
  int data_len;
  uint16_t index;
  uint8_t sub_index;
  uint16_t data;

  // Package 1
  index = 0x6060; sub_index = 0x00; data = 0x06;
  pkg_len = 8; data_len = 1;
  SDOwriteWrapper_(pkg_len, node, index, sub_index);
  SDOdataWrapper_(data_len, data);
  calcCRC_(); 
  //showPackage_((uint8_t *)obuff_, pkg_len + 2);
  serial_port_.write((char *)obuff_, pkg_len + 2);
  consumeResponse_();

  // Package 2
  index = 0x6098; sub_index = 0x00; data = 0x25;
  pkg_len = 8; data_len = 1;
  SDOwriteWrapper_(pkg_len, node, index, sub_index);
  SDOdataWrapper_(data_len, data);
  calcCRC_(); 
  //showPackage_((uint8_t *)obuff_, pkg_len + 2);
  serial_port_.write((char *)obuff_, pkg_len + 2);
  consumeResponse_();

  // Package 3
  index = 0x6040; sub_index = 0x00; data = 0x0E;
  pkg_len = 9; data_len = 2;
  SDOwriteWrapper_(pkg_len, node, index, sub_index);
  SDOdataWrapper_(data_len, data);
  calcCRC_(); 
  //showPackage_((uint8_t *)obuff_, pkg_len + 2);
  serial_port_.write((char *)obuff_, pkg_len + 2);
  consumeResponse_();

  // Package 4
  index = 0x6040; sub_index = 0x00; data = 0x0F;
  pkg_len = 9; data_len = 2;
  SDOwriteWrapper_(pkg_len, node, index, sub_index);
  SDOdataWrapper_(data_len, data);
  calcCRC_(); 
  //showPackage_((uint8_t *)obuff_, pkg_len + 2);
  serial_port_.write((char *)obuff_, pkg_len + 2);
  consumeResponse_();

  // Package 5
  index = 0x6040; sub_index = 0x00; data = 0x1F;
  pkg_len = 9; data_len = 2;
  SDOwriteWrapper_(pkg_len, node, index, sub_index);
  SDOdataWrapper_(data_len, data);
  calcCRC_(); 
  //showPackage_((uint8_t *)obuff_, pkg_len + 2);
  serial_port_.write((char *)obuff_, pkg_len + 2);
  consumeResponse_();
}

void SerialHandle::update(int32_t vel1, int32_t vel2)
{
  static int enc_counter = 0;
  int enc_max_cycle = 1;
  char next_byte;
  int count = 0;

  SDOwriteWrapper_(11, 1, 0x60FF, 0x00);
  SDOS32dataWrapper_(vel1);
  calcCRC_(); 
  serial_port_.write((char *)obuff_, 13);
  //std::cout << "node" << node << " send: ";
  //showPackage_((uint8_t *)obuff_, pkg_len + 2);

  count = 0;
  while(1)
  {
    serial_port_.get(next_byte);
    ibuff_[count++] = next_byte;
    if(next_byte == 0x45)
      break;
  }

  if((uint8_t)ibuff_[0] == 0x53 && 
     (uint8_t)ibuff_[1] == 0x0B && count == 13)
  {
    std::cout << "node" << (uint8_t)ibuff_[2] << " receive: ";
    showPackage_((uint8_t *)ibuff_, count);
  }

  //std::cout << "node" << node << " receive: ";
  //showPackage_((uint8_t *)ibuff_, count);

  SDOwriteWrapper_(11, 2, 0x60FF, 0x00);
  SDOS32dataWrapper_(vel2);
  calcCRC_(); 
  serial_port_.write((char *)obuff_, 13);
  //std::cout << "node" << node << " send: ";
  //showPackage_((uint8_t *)obuff_, pkg_len + 2);

  count = 0;
  while(1)
  {
    serial_port_.get(next_byte);
    ibuff_[count++] = next_byte;
    if(next_byte == 0x45)
      break;
  }
  if((uint8_t)ibuff_[0] == 0x53 && 
     (uint8_t)ibuff_[1] == 0x0B && count == 13)
  {
    std::cout << "node" << (uint8_t)ibuff_[2] << " receive: ";
    showPackage_((uint8_t *)ibuff_, count);
  }

  //std::cout << "node" << node << " receive: ";
  //showPackage_((uint8_t *)ibuff_, count);

  if(enc_counter == enc_max_cycle)
  {
    //std::cout << "enc" << std::endl;
    // Read encoder
    SDOreadWrapper_(1, 0x6064, 0x00);
    calcCRC_(); 
    serial_port_.write((char *)obuff_, READ_CMD_LEN + 2);
    //std::cout << "node" << node << " send: ";
    //showPackage_((uint8_t *)obuff_, READ_CMD_LEN + 2);

    count = 0;
    while(1)
    {
      serial_port_.get(next_byte);
      ibuff_[count++] = next_byte;
      if(next_byte == 0x45)
        break;
    }

    if((uint8_t)ibuff_[0] == 0x53 && 
       (uint8_t)ibuff_[1] == 0x0B && 
        count == 13)
    {
      std::cout << "node" << (uint8_t)ibuff_[2] << " receive: ";
      showPackage_((uint8_t *)ibuff_, count);
    }

    //std::cout << "node" << node << " receive: ";
    //showPackage_((uint8_t *)ibuff_, count);

    SDOreadWrapper_(2, 0x6064, 0x00);
    calcCRC_(); 
    serial_port_.write((char *)obuff_, READ_CMD_LEN + 2);
    //std::cout << "node" << node << " send: ";
    //showPackage_((uint8_t *)obuff_, READ_CMD_LEN + 2);

    count = 0;
    while(1)
    {
      serial_port_.get(next_byte);
      ibuff_[count++] = next_byte;
      if(next_byte == 0x45)
        break;
    }

    if((uint8_t)ibuff_[0] == 0x53 && 
       (uint8_t)ibuff_[1] == 0x0B && 
        count == 13)
    {
      std::cout << "node" << (uint8_t)ibuff_[2] << " receive: ";
      showPackage_((uint8_t *)ibuff_, count);
    }
    enc_counter = 0;
  }
  else
  {
    //std::cout << "no enc" << std::endl;
    enc_counter ++;
  }
}

#if 1
void SerialHandle::writeSpeed(int node, int32_t vel)
{
  char next_byte;
  int count = 0;
  int pkg_len = 11;
  int data_len = 4;
  uint16_t index = 0x60FF;
  uint8_t sub_index = 0x00;

  SDOwriteWrapper_(pkg_len, node, index, sub_index);
  SDOS32dataWrapper_(vel);
  //SDOdataWrapper_(data_len, vel);
  calcCRC_(); 
  serial_port_.write((char *)obuff_, pkg_len + 2);
  //std::cout << "node" << node << " send: ";
  //showPackage_((uint8_t *)obuff_, pkg_len + 2);


  while(1)
  {
    serial_port_.get(next_byte);
    if(next_byte == 0x53)
    {
      ibuff_[count++] = next_byte;
      break;
    }
  }
  while(1)
  {
    serial_port_.get(next_byte);
    ibuff_[count++] = next_byte;
    if(next_byte == 0x45)
      break;
  }
  //std::cout << "node" << node << " receive: ";
  //showPackage_((uint8_t *)ibuff_, count);

#if 0
  if((uint8_t)ibuff_[0] == 0x53 && 
     (uint8_t)ibuff_[1] == 0x0B && count == 13)
  {
    // Rebuild encoder values
    for(int i = 0; i < 4; i++)
      act_pos_buff[i] = ibuff_[i + 7];
    act_pos = ((int *)act_pos_buff)[0];

    std::cout << "node" << node << " receive: ";
    std::cout << act_pos << std::endl;
    stuck = false;
  }
#endif

  //std::cout << "node" << node << " receive: ";
  //showPackage_((uint8_t *)ibuff_, count);


}
#endif

int SerialHandle::readEnc(int node)
{
  uint16_t index = 0x6064;
  uint8_t sub_index = 0x00;
  char act_pos_buff[4];
  int act_pos = 0;
  int count = 0;
  char next_byte;

  SDOreadWrapper_(node, index, sub_index);
  calcCRC_(); 
  serial_port_.write((char *)obuff_, READ_CMD_LEN + 2);
  //std::cout << "node" << node << " send: ";
  //showPackage_((uint8_t *)obuff_, READ_CMD_LEN + 2);

  while(1)
  {
    serial_port_.get(next_byte);
    if(next_byte == 0x53)
    {
      ibuff_[count++] = next_byte;
      break;
    }
  }
  while(1)
  {
    serial_port_.get(next_byte);
    ibuff_[count++] = next_byte;
    if(next_byte == 0x45)
      break;
  }
  //std::cout << "node" << node << " receive: ";
  //showPackage_((uint8_t *)ibuff_, count);

  if((uint8_t)ibuff_[0] == 0x53 && 
     (uint8_t)ibuff_[1] == 0x0B && 
     (uint8_t)ibuff_[2] == node && 
      count == 13)
  {
    // Rebuild encoder values
    for(int i = 0; i < 4; i++)
      act_pos_buff[i] = ibuff_[i + 7];
    act_pos = ((int *)act_pos_buff)[0];

    //std::cout << "node" << node << " receive: ";
    //std::cout << act_pos << std::endl;
    return act_pos;
  }
  else
    throw std::runtime_error("incomplete enc info.");
    //std::cout << "pass" << std::endl;
}

bool SerialHandle::getActPosPkg_(int node)
{
  uint16_t index = 0x6064;
  uint8_t sub_index = 0x00;
  char ibuff_temp[16];
  char next_byte;
  int pkg_len;

  SDOreadWrapper_(node, index, sub_index);
  calcCRC_(); 
  serial_port_.write((char *)obuff_, READ_CMD_LEN + 2);

  // Wait for 'S'
  serial_port_.get(next_byte);
  if(next_byte == 'S')
  {
    // Get package length and start reading the whole block
    serial_port_.get(next_byte);
    pkg_len = (int)next_byte;
    serial_port_.read(ibuff_temp, pkg_len);
  }

  // Rebuild the package
  ibuff_[0] = 0x53;
  ibuff_[1] = (uint8_t)pkg_len;
  for(int i = 0; i < pkg_len; i++)
  {
    ibuff_[i + 2] = ibuff_temp[i];
  }
  //showPackage_((uint8_t *)ibuff_, pkg_len + 2);

  // Verify CRC
  if(!verifyCRC_())
  {
    ROS_WARN("Fail to read compact CANopen packages. The signals may be interfered.");
    return false;
  }

  // Check whether this is actual position package
  if((uint8_t)ibuff_[4] != 0x64 || (uint8_t)ibuff_[5] != 0x60)
  {
    return false;
  }

  return true;
}



#if 1
void SerialHandle::writeData(std::string p_data)
{
  serial_port_ << p_data << std::endl ;
}

std::string SerialHandle::readData()
{
  char next_byte;
  std::stringstream ss_enc;
  ss_enc.str("");
  while(ros::ok()) 
  {
    serial_port_.get(next_byte);
    if(next_byte == '\r')
      break;
    else
        ss_enc << next_byte;
  } 

  return ss_enc.str();
}
#endif

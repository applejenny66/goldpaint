#include <iostream>
#include <boost/thread/thread.hpp>
#include "scorpio_uart/scorpio_uart.h"
#include "ros/ros.h"

#if 0
typedef boost::shared_lock<boost::shared_mutex> readLock;  
typedef boost::unique_lock<boost::shared_mutex> writeLock;
boost::shared_mutex rwmutex; 

int temp[9];

void write_odom(SerialHandle *my_port, int *temp)
{
  //ros::Rate rate(10);
  while(ros::ok())
  {
    //std::cout << "origin: " << int(my_port->readData()) << std::endl;
#if 1
    if(int(my_port->readData()) == -22)
      for(int i = 0; i < 9; i++)
      {
        writeLock wtlock(rwmutex);  
        temp[i] = int(my_port->readData());
        //std::cout << "after: " << temp[i] << std::endl;
      }
#endif
    //rate.sleep();
  }
}
#endif

#if 0
void read_odom(SerialHandle *my_port, int *temp)
{

  //ros::Rate rate(10);
  //while(ros::ok())
  {
    readLock rdlock(rwmutex); 
    std::cout << "=====" << std::endl;
    for(int i = 0; i < 9; i++)
      std::cout << temp[i] << std::endl;
    //rate.sleep();
  }
}
#endif

#if 0
void read_odom_once()
{
  readLock rdlock(rwmutex); 
  std::cout << "=====" << std::endl;
  for(int i = 0; i < 9; i++)
    std::cout << temp[i] << std::endl;
}
#endif

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scorpio_uart_node");
  ros::NodeHandle nh;

  SerialHandle my_port; 
  my_port.set_port("/dev/ttyUSB0");
  my_port.set_baudRate(9600);
  my_port.set_dataBits(8);
  my_port.set_stopBit(1);
  my_port.set_parity(false);
  my_port.set_hardwareFlowControl(false);

  my_port.writeSpeed(2, -100);

  sleep(1);

  my_port.writeSpeed(2, 0);
  //my_port.readEnc(2);
  //my_port.writeSpeed(2, 0);
#if 0
  my_port.writeSpeed(2, 500);
  //my_port.writeSpeed(2, 1000);

  ros::Rate r(10);
  int counter = 0;
  while(ros::ok() && counter < 20)
  {
    std::cout << my_port.readEnc(2) << std::endl;
    counter ++;
    r.sleep();
  }

  my_port.writeSpeed(2, 0);
#endif

  //sleep(5);

  //my_port.writeSpeed(1, 0);
  //my_port.writeSpeed(2, 0);

#if 0
  my_port.switchOn(1);
  sleep(1);

  std::cout << "homing" << std::endl;
  my_port.homing(1);
  sleep(1);

  //my_port.switchOff(1);
  //sleep(1);

  //my_port.switchOn(1);
  //sleep(1);
  //my_port.writeSpeed(1, 10);

  ros::Rate r(10);
#if 0
  int counter = 0;
  while(ros::ok() && counter < 20)
  {
    std::cout << my_port.readEnc(1) << std::endl;
    counter ++;
    r.sleep();
  }
#endif


#if 1
  while(ros::ok())
  {
    std::cout << my_port.readEnc(1) << std::endl;
    r.sleep();
  }
#endif

#endif
#if 0
  my_port.switchOn(1);
  sleep(1);

  ros::Rate r(10);
  int speed = 100;
  while(ros::ok() && speed < 1000)
  {
    my_port.writeSpeed(1, speed);
    my_port.readEnc(1);
    speed += 10;
    r.sleep();
  }

  while(ros::ok() && speed > 0)
  {
    my_port.writeSpeed(1, speed);
    my_port.readEnc(1);
    speed -= 10;
    r.sleep();
  }

  my_port.switchOff(1);
#endif
  //my_port.SDOwriteWrapper(9, 1, 0x6040, 0x00);
  //char arr[] = {'a', 'p', 'p', 'l', 'e'};
  //uint8_t arr[16] = {0xFF, 0x01, 0x5D, 0x6C, 0x32};
#if 0
  uint8_t arr[16];
  arr[0] = 0x53;
  arr[1] = 0x09;
  arr[2] = 0x01;
  arr[3] = 0x02;
  arr[4] = 0x40;
  arr[5] = 0x60;
  arr[6] = 0x00;
  arr[7] = 0x0D;
  arr[8] = 0x00;
  my_port.CalcCRC(arr);
#endif

#if 0
  uint8_t u8CRC;
  u8CRC = my_port.CalcCRCByte(0x09, 0xFF);
  u8CRC = my_port.CalcCRCByte(0x01, u8CRC);
  u8CRC = my_port.CalcCRCByte(0x02, u8CRC);
  u8CRC = my_port.CalcCRCByte(0x40, u8CRC);
  u8CRC = my_port.CalcCRCByte(0x60, u8CRC);
  u8CRC = my_port.CalcCRCByte(0x00, u8CRC);
  u8CRC = my_port.CalcCRCByte(0x0D, u8CRC);
  u8CRC = my_port.CalcCRCByte(0x00, u8CRC);
#endif

#if 0
  u8CRC = my_port.CalcCRCByte(0x00, 0xFF);
  u8CRC = my_port.CalcCRCByte(0x0E, u8CRC);
  u8CRC = my_port.CalcCRCByte(0x00, u8CRC);
  u8CRC = my_port.CalcCRCByte(0x60, u8CRC);
  u8CRC = my_port.CalcCRCByte(0x40, u8CRC);
  u8CRC = my_port.CalcCRCByte(0x02, u8CRC);
  u8CRC = my_port.CalcCRCByte(0x01, u8CRC);
  u8CRC = my_port.CalcCRCByte(0x09, u8CRC);
#endif

#if 0
  u8CRC = my_port.CalcCRCByte(0x00, 0xFF);
  u8CRC = my_port.CalcCRCByte(0x60, u8CRC);
  u8CRC = my_port.CalcCRCByte(0x64, u8CRC);
  u8CRC = my_port.CalcCRCByte(0x01, u8CRC);
  u8CRC = my_port.CalcCRCByte(0x02, u8CRC);
  u8CRC = my_port.CalcCRCByte(0x07, u8CRC);
#endif

#if 0
  u8CRC = my_port.CalcCRCByte(0x07, 0xFF);
  u8CRC = my_port.CalcCRCByte(0x02, u8CRC);
  u8CRC = my_port.CalcCRCByte(0x01, u8CRC);
  u8CRC = my_port.CalcCRCByte(0x64, u8CRC);
  u8CRC = my_port.CalcCRCByte(0x60, u8CRC);
  u8CRC = my_port.CalcCRCByte(0x00, u8CRC);
#endif

  //dec_temp = u8CRC;
  //std::cout << dec_temp << std::endl;
  //std::cout << u8CRC << std::endl;
#if 0
#endif
  //dec_temp = u8CRC;
  //std::cout << dec_temp << std::endl;
  //std::cout << u8CRC << std::endl;

#if 0
  // Switch on
  my_port.switchOn();

  sleep(1);
  my_port.writeSpeed();

  sleep(1);
  std::cout << "reading enc..." << std::endl;
  my_port.readEnc();

  sleep(1);
  my_port.switchOff();
#endif
#if 0
  //boost::thread thread_read(read_odom, &my_port, temp);
  boost::thread thread_write(write_odom, &my_port, temp);

  ros::Rate rate(10);
#endif

#if 0
  read_odom_once();
  sleep(1);

  read_odom_once();
  sleep(1);

  read_odom_once();
  sleep(1);

  read_odom_once();
  sleep(1);
#endif

#if 0
  while(ros::ok())
  {
    read_odom_once();
    rate.sleep();
  }
#endif

  //thread_read.join();
  //thread_write.join();

#if 0
  int temp;
  while(1)
  {
    temp = int(my_port.readData());
    std::cout << temp << std::endl;
  } 
#endif
#if 0
  my_port.writeData(100, 300);
  sleep(5);

  my_port.writeData(-100, -300);
  sleep(5);

  my_port.writeData(300, 100);
  sleep(5);

  my_port.writeData(-300, -100);
  sleep(5);
#endif

#if 0
  std::cout << "100" << std::endl;
  my_port.writeData(-100, -100);
  sleep(5);

  std::cout << "200" << std::endl;
  my_port.writeData(-200, -200);
  sleep(5);

  std::cout << "250" << std::endl;
  my_port.writeData(-250, -250);
  sleep(5);

  std::cout << "300" << std::endl;
  my_port.writeData(-300, -300);
  sleep(5);

  std::cout << "400" << std::endl;
  my_port.writeData(-400, -400);
  sleep(5);

  std::cout << "500" << std::endl;
  my_port.writeData(-500, -500);
  sleep(5);

  std::cout << "0" << std::endl;
  my_port.writeData(0, 0);
  sleep(5);
#endif

#if 0
  my_port.writeData(100, -100);
  sleep(1);

  my_port.writeData(-100, 100);
  sleep(1);

  my_port.writeData(200, -200);
  sleep(1);

  my_port.writeData(-200, 200);
  sleep(1);

  my_port.writeData(300, -300);
  sleep(1);

  my_port.writeData(-300, 300);
  sleep(1);

  my_port.writeData(200, -200);
  sleep(1);

  my_port.writeData(-200, 200);
  sleep(1);

  my_port.writeData(100, -100);
  sleep(1);

  my_port.writeData(-100, 100);
  sleep(1);

  my_port.writeData(0, 0);
  sleep(1);
#endif

}

#ifndef SCORPIO_UART 
#define SCORPIO_UART 

#include <sstream>
#include <string>
#include <SerialStream.h>

using namespace LibSerial ;    

class SerialHandle
{
  public:
    SerialHandle();
    ~SerialHandle();
    void set_port(const char* const);
    void set_baudRate(int);
    void set_dataBits(int);
    void set_stopBit(int);
    void set_parity(bool);
    void set_hardwareFlowControl(bool);
    void writeData(std::string p_data);
    void switchOn(int);
    void switchOff(int);
    void homing(int);
    void writeSpeed(int, int32_t);
    void update(int32_t, int32_t);
    int readEnc(int);
    std::string readData();

    
  private:
    SerialStream serial_port_;
    //const int OBUFFER_SIZE = 11;
    const int DATA_ADDR = 7;
    //const int IBUFF_LEN = 16;
    const int READ_CMD_LEN = 7;
    int baud_rate_;
    int data_bits_;
    int stop_bit_;
    bool flow_control_;
    //char output_buffer[OBUFFER_SIZE];
    //uint8_t output_buffer_[16];
    uint8_t obuff_[16];
    char ibuff_[16];
    //uint8_t input_buffer_[16];
    void calcCRC_();
    bool verifyCRC_();
    uint8_t calcCRCByte_(uint8_t, uint8_t);
    void SDOdataWrapper_(int, uint16_t);
    void SDOS32dataWrapper_(int32_t);
    void SDOwriteWrapper_(int, int, uint16_t, uint8_t);
    void SDOreadWrapper_(int, uint16_t, uint8_t);
    void showPackage_(uint8_t *, int);
    bool getActPosPkg_(int);
    void consumeResponse_();
};

#endif

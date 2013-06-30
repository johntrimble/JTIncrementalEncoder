#ifndef interface_h
#define interface_h

#include <Wire.h>
#include <EEPROM.h>
#include <MCP42xxx.h>
#include "encoder.h"

namespace JTIncrementalEncoder {

template <typename WIRE, typename ENCODER>
class EncoderInterface {
  WIRE& twoWire;
  ENCODER& encoder;
  uint8_t registers[NUMBER_REGISTERS];
  uint8_t receivedData[MAX_WRITE_BYTES];
  uint8_t bytesReceivedFromMaster;

public:
  EncoderInterface(WIRE& twoWire, ENCODER& encoder);
  void begin(uint8_t slaveAddress);

  void update(EncoderState& state);
  void update();

  uint8_t getCalibrationMode() { 
    return this->registers[MODE_INDEX]&CALIBRATION_MODE; 
  }

  void setCalibrationMode(uint8_t val) {
    if( val )
      this->registers[MODE_INDEX] |= CALIBRATION_MODE;
    else
      this->registers[MODE_INDEX] &= ~CALIBRATION_MODE;
  }

  uint8_t getResetHomeMode() {
    return this->registers[MODE_INDEX]&RESET_HOME_MODE;
  }

  void setResetHomeMode(uint8_t val) {
    if( val )
      this->registers[MODE_INDEX] |= RESET_HOME_MODE;
    else
      this->registers[MODE_INDEX] &= ~RESET_HOME_MODE;
  }

  uint8_t getCalibratedStatus() {
    return this->registers[STATUS_REGISTER] & UNCALIBRATED_STATUS ? 1 : 0;
  }

  void setCalibratedStatus(uint8_t val) {
    if( val ) {
      this->registers[STATUS_REGISTER] &= ~UNCALIBRATED_STATUS;
    } else {
      this->registers[STATUS_REGISTER] |= UNCALIBRATED_STATUS;
    }
  }
  
  void requestEvent();
  void receiveEvent(int bytesReceived);
};

}

#endif
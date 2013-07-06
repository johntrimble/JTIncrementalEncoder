#ifndef interface_h
#define interface_h

#include "encoder.h"

namespace JTIncrementalEncoder {


// There are 7 single byte registers:
// 0 - status register (read only)
// 1 - error register (read only)
// 2 - state register (read only)
// 3 - position register (read only)
// 4 - mode register (read/write)
// 5 - divisions register (read/write)
// 6 - identification (read only)

const uint8_t NUMBER_REGISTERS = 7;
const uint8_t MAX_WRITE_BYTES = 2;

// Mode register
// D7 D6 D5 D4 D3 D2 D1 D0 
// X  X  X  X  X  CM RH TM
//
// CM - Calibration Mode (1 means calibrating)
// RH - Reset Home (1 means resetting home position)
// TM - Tracking Mode (1 means currently tracking)
const uint8_t CALIBRATION_MODE = 0x04; // B00000100
const uint8_t RESET_HOME_MODE  = 0x02; // B00000010
const uint8_t TRACKING_MODE    = 0x01; // B00000001

// Status register
// D7 D6 D5 D4 D3 D2 D1 D0 
// X  X  X  X  X  X  BS CS
//
// BS - Busy Status (1 means busy)
// CS - Calibration Status (1 means uncalibrated)
const uint8_t READY_STATUS        = 0x00;
const uint8_t BUSY_STATUS         = 0x02; // B00000010
const uint8_t UNCALIBRATED_STATUS = 0x01; // B00000001

const uint8_t STATUS_REGISTER = 0;
const uint8_t ERROR_REGISTER = 1;
const uint8_t STATE_REGISTER = 2;
const uint8_t POSITION_REGISTER = 3;
const uint8_t MODE_INDEX = 4;
const uint8_t DIVISION_REGISTER = 5;
const uint8_t IDENTIFICATION_REGISTER = 6;


template <typename WIRE, typename ENCODER>
class EncoderInterface {
  static EncoderInterface<WIRE,ENCODER>* singleton;

  WIRE& twoWire;
  ENCODER& encoder;
  uint8_t registers[NUMBER_REGISTERS];
  uint8_t receivedData[MAX_WRITE_BYTES];
  uint8_t bytesReceivedFromMaster;

public:
  EncoderInterface(WIRE& twoWire, ENCODER& encoder);
  void begin(uint8_t slaveAddress);

  void update(const EncoderState& state);
  void update(); 
  
  static void requestEvent();
  static void receiveEvent(int bytesReceived);

  inline static const EncoderInterface<WIRE,ENCODER>* getSingleton() {
    return EncoderInterface<WIRE,ENCODER>::singleton;
  }

  inline uint8_t getCalibrationMode() const {
    return this->registers[MODE_INDEX]&CALIBRATION_MODE;
  }

  inline void setCalibrationMode(uint8_t val) {
    if( val )
      this->registers[MODE_INDEX] |= CALIBRATION_MODE;
    else
      this->registers[MODE_INDEX] &= ~CALIBRATION_MODE;
  }

  inline uint8_t getResetHomeMode() const {
    return this->registers[MODE_INDEX]&RESET_HOME_MODE;
  }

  inline void setResetHomeMode(uint8_t val) {
    if( val )
      this->registers[MODE_INDEX] |= RESET_HOME_MODE;
    else
      this->registers[MODE_INDEX] &= ~RESET_HOME_MODE;
  }

  inline uint8_t getBusyStatus() const {
    return this->registers[STATUS_REGISTER]&BUSY_STATUS;
  }

  inline void setBusyStatus(uint8_t val) {
    if( val ) {
      this->registers[STATUS_REGISTER] |= BUSY_STATUS;
    } else {
      this->registers[STATUS_REGISTER] &= ~BUSY_STATUS;
      this->bytesReceivedFromMaster = 0;
    }
  }

  inline uint8_t getCalibratedStatus() const {
    return this->registers[STATUS_REGISTER] & UNCALIBRATED_STATUS ? 1 : 0;
  }

  inline void setCalibratedStatus(uint8_t val) {
    if( val ) {
      this->registers[STATUS_REGISTER] &= ~UNCALIBRATED_STATUS;
    } else {
      this->registers[STATUS_REGISTER] |= UNCALIBRATED_STATUS;
    }
  }
};

}

#endif
#ifndef interface_h
#define interface_h

#include <vector>
#include <Wire.h>
#include "encoder.h"

namespace JTIncrementalEncoder {

class EncoderInterface {
  TwoWire& twoWire;
  uint8_t registers[NUMBER_REGISTERS];
  uint8_t receivedData[MAX_WRITE_BYTES];
  uint8_t bytesReceivedFromMaster;
  std::vector<uint8_t> calibration_mode_sequence_values;

public:
  EncoderInterface(TwoWire& twoWire);
  void begin(uint8_t slaveAddress);

  void update(EncoderState& state);
  void update();

  uint8_t getCalibrationMode();
  void setCalibrationMode(uint8_t);

  void set_calibration_mode_sequence(std::vector<uint8_t> &values);

  uint8_t getResetHomeMode();
  void setResetHomeMode(uint8_t);

  uint8_t getCalibratedStatus();
  void setCalibratedStatus(uint8_t val);
  
  void requestEvent();
  void receiveEvent(int bytesReceived);
};

extern EncoderInterface EncoderInterface;

}
#endif
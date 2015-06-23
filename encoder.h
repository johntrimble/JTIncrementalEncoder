#ifndef encoder_h
#define encoder_h

#include <inttypes.h>
#include "Arduino.h"
#include "jt_calibrator.h"

namespace JTIncrementalEncoder {

const int NUMBER_CHANNELS = 2;
const int DIVISIONS = 92;

// Direction
const uint8_t CLOCKWISE = 0;
const uint8_t COUNTERCLOCKWISE = 1;

typedef struct {
  int inputPin;
  int interrupt;
  int average;
  int minValue;
  int maxValue;
  void (*isrFunc)(void);
} EncoderChannel;

typedef struct {
  EncoderChannel a;
  EncoderChannel b;
} EncoderChannelPair;

typedef struct {
  EncoderChannelPair channels;
  uint8_t calibrated;

  // D7 D6 D5 D4 D3 D2 D1 D0
  // A1 B1 A0 B0 I3 I2 I1 I0
  // 
  // A1: the previous value of channel A
  // B1: the previous value of channel B
  // A0: the current value of channel A
  // B0: the current value of channel B
  // D0-D3: the current index, incremented on every state transition.
  volatile uint8_t encoderState;
  int position;
} EncoderState;

template<typename CALIBRATOR>
struct Calibration {
  CALIBRATOR& a;
  CALIBRATOR& b;
  Calibration(CALIBRATOR& a, CALIBRATOR& b)
    : a(a), b(b) { 
  }
};

template <typename CALIBRATOR, typename STORAGE, typename LOG>
class Encoder {
  Calibration<CALIBRATOR>& calibration;
  STORAGE& storage;
  LOG& log;
  uint8_t calibrating;
  uint8_t revolutions;

  void updateVoltageReference();
  void positionDidChange(
    const uint8_t& revolutionCompleted,
    const uint8_t& revolutions,
    const uint8_t& previousDirection, 
    const int& previousPosition, 
    const uint8_t& currentDirection, 
    const int& currentPosition);
public:
  // TODO: make the 'state' instance variable private and fix whatever is causing it to be public.
  EncoderState state;
  Encoder(
    STORAGE& EEPROM,
    LOG& log,
    Calibration<CALIBRATOR>& calibration,
    uint8_t channelAInputPin,
    uint8_t channelAInterrupt,
    void (*channelAIsrFunc)(void),
    uint8_t channelBInputPin,
    uint8_t channelBInterrupt,
    void (*channelBIsrFunc)(void));

  const EncoderState& getState() const;
  uint8_t getDirection(uint8_t encoderState);
  void resetPosition();
  void startCalibration();
  void stopCalibration();
  uint8_t saveSettings(STORAGE& storage, int addr);
  uint8_t loadSettings(STORAGE& storage, int addr);
  void update();
};

template <typename PRINT>
inline void printChannelInfo(PRINT& p, EncoderChannel& channel) {
  p.print("Average: "); p.println(channel.average);
  p.print("Min: "); p.println(channel.minValue);
  p.print("Max: "); p.println(channel.maxValue);
}

static inline void updateChannelAEncoderState(const EncoderChannel& channel, volatile uint8_t& encoderState, uint8_t& isrSteps) {
  encoderState = ((encoderState << 2)&0xC0/*B11000000*/)|(encoderState&0x20/*B00100000*/)|(digitalRead(channel.inputPin)<<4)|(isrSteps&0x0F/*B00001111*/);
  isrSteps++;
}

static inline void updateChannelBEncoderState(const EncoderChannel& channel, volatile uint8_t& encoderState, uint8_t& isrSteps) {
  encoderState = ((encoderState << 2)&0xC0/*B11000000*/)|(encoderState&0x10/*B00010000*/)|(digitalRead(channel.inputPin)<<5)|(isrSteps&0x0F/*B00001111*/);
  isrSteps++;
}

}

#endif

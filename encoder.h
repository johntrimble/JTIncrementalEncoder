#ifndef encoder_h
#define encoder_h

#include <MCP42xxx.h>
#include "Arduino.h"
#include "inttypes.h"
#include "interface.h"

namespace JTIncrementalEncoder {

const uint8_t NUMBER_REGISTERS = 7;
const uint8_t MAX_WRITE_BYTES = 2;

const int NUMBER_CHANNELS = 2;
const int DIVISIONS = 92;

typedef struct {
  int inputPin;
  int rawInputPin;
  int interrupt;
  int average;
  int minValue;
  int maxValue;
  MCP42xxx::Channel channel;
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
  // B1: the current value of channel B
  // D0-D3: the current index, incremented on every state transition.
  volatile uint8_t encoderState;
  int position;
} EncoderState;

// Direction
const uint8_t CLOCKWISE = 0;
const uint8_t COUNTERCLOCKWISE = 1;

uint8_t loadSettings(int addr, EncoderChannelPair& channels);
uint8_t saveSettings(int addr, const EncoderChannelPair& channels);
  
void isort(int arr[], int length);
uint8_t getDirection(uint8_t encoderState);
uint8_t sample(uint8_t pin);
void runCalibration(EncoderChannelPair& channelPair);
uint8_t updatePosition(uint8_t previousState, uint8_t currentState, int &position);

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

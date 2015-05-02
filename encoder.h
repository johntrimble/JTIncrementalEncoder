#ifndef encoder_h
#define encoder_h

#include <inttypes.h>
#include <MCP42xxx.h>
#include "Arduino.h"

namespace JTIncrementalEncoder {

const int NUMBER_CHANNELS = 2;
const int DIVISIONS = 92;

// Direction
const uint8_t CLOCKWISE = 0;
const uint8_t COUNTERCLOCKWISE = 1;

const uint8_t MOVING_AVERAGE_DATA_SIZE = 16; // 2^4
const unsigned long DUTY_CYCLE_MA_SAMPLE_LENGTH = 2e17;

typedef struct {
  uint8_t* head;
  uint8_t* data_end;
  uint8_t average;
  uint8_t ready;
  uint8_t data[MOVING_AVERAGE_DATA_SIZE]; 
} MovingAverage;

void initMovingAverage(MovingAverage& buffer) {
  buffer.head = buffer.data;
  buffer.data_end = buffer.data + MOVING_AVERAGE_DATA_SIZE;
  buffer.average = 0;
  buffer.ready = 0;
  for( uint8_t* i = buffer.data; i != buffer.data_end; i++) {
    *i = 0;
  }
}

uint8_t getHeadValueMovingAverage(MovingAverage& buffer) {
  return *(buffer.head);
}

uint8_t getEndValueMovingAverage(MovingAverage& buffer) {
  uint8_t *end = buffer.head - 1;
  if( end < buffer.data ) {
    end = buffer.data_end - 1;
  }
  return *end;
}

void addToMovingAverage(MovingAverage& buffer, uint8_t value) {
  buffer.head++;
  if( buffer.head == buffer.data_end ) {
    buffer.head = buffer.data;
    buffer.ready = 1;
  }
  *(buffer.head) = value;
  // NOTE: subtraction first to prevent overflow!
  buffer.average = (buffer.average - (getEndValueMovingAverage(buffer) >> 4))
      + (value >> 4);
}

uint8_t currentMovingAverage(MovingAverage& buffer) {
  return buffer.average;
}

uint8_t isReadyMovingAverage(MovingAverage& buffer) {
  return buffer.ready;
}

typedef struct {
  unsigned lastUpdateTime;
  unsigned long acc;

  MovingAverage movingAverage;
} DutyCycleMovingAverage;

void initDutyCycleMovingAverage(
  DutyCycleMovingAverage& dutyCycleMA,
  unsigned long currentTimeMicros) {
  initMovingAverage(dutyCycleMA.movingAverage);
  dutyCycleMA.lastUpdateTime = currentTimeMicros;
}

void updateDutyCycleMA(
  DutyCycleMovingAverage& dutyCycleMA, 
  uint8_t channelValue, 
  unsigned long currentTimeMicros) {

  unsigned long delta;
  if( currentTimeMicros < dutyCycleMA.lastUpdateTime) {
    delta = (ULONG_MAX - dutyCycleMA.lastUpdateTime) + currentTimeMicros;
  } else {
    delta = currentTimeMicros - dutyCycleMA.lastUpdateTime;
  }

  if( channelValue == HIGH )
    dutyCycleMA.acc += delta;

  if( delta > DUTY_CYCLE_MA_SAMPLE_LENGTH ) {
    dutyCycleMA.lastUpdateTime = currentTimeMicros;
    // the max value of acc is 2^17 (our sample length). We want an 8 bit 
    // number, so just take 8 most significant bits of our 17 bit number.
    uint8_t value = (dutyCycleMA.acc >> 9) & 0xFF;
    addToMovingAverage(dutyCycleMA.movingAverage, value);
    dutyCycleMA.lastUpdateTime = currentTimeMicros;
    dutyCycleMA.acc = 0;
  }
}

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
  // B0: the current value of channel B
  // D0-D3: the current index, incremented on every state transition.
  volatile uint8_t encoderState;
  int position;
} EncoderState;

const uint8_t SAMPLE_COUNT = 5;
const unsigned long SAMPLE_TIMEOUT = 5000000; // 5 seconds

typedef struct {
  unsigned long lowTime;
  unsigned long highTime;
} SamplingResult;

typedef struct {
  unsigned long duration;
  unsigned long startTime;
  uint8_t finished;
  uint8_t type;
} Sample;

typedef struct { 
  uint8_t type;
  uint8_t samplesIndex;
  Sample samples[SAMPLE_COUNT];
} SamplingData;

typedef struct {
  uint8_t mask;
  uint8_t potSetting;
  uint8_t finished;
  SamplingData samplingData;
} ChannelCalibration;

typedef struct {
  ChannelCalibration a;
  ChannelCalibration b;
} Calibration;

template <typename POT, typename STORAGE, typename LOG>
class Encoder {
  Calibration calibration;
  POT& pot;
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
    POT& pot,
    STORAGE& EEPROM,
    LOG& log,
    uint8_t channelAInputPin, 
    uint8_t channelARawInputPin,
    MCP42xxx::Channel channelApotChannel,
    uint8_t channelAInterrupt,
    void (*channelAIsrFunc)(void),
    uint8_t channelBInputPin,
    uint8_t channelBRawInputPin,
    MCP42xxx::Channel channelBpotChannel,
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

uint8_t sample(uint8_t pin);
void isort(int arr[], int length);


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

#include <limits.h>
#include <EEPROM.h>
#include "encoder.h"

namespace JTIncrementalEncoder {

static const int INDEX_INDICATOR_PIN = 7;

template<typename POT>
static inline void updateCalibration(MCP42xxx::Channel& channelA, MCP42xxx::Channel& channelB, POT& pot, Calibration& calibration, uint8_t encoderState);
static inline void stopCalibration(EncoderChannelPair& channelPair);
static uint8_t getDirection(uint8_t encoderState);
static uint8_t updatePosition(uint8_t previousState, uint8_t currentState, int &position);
static uint8_t getCurrentChannelValue(uint8_t encoderState, uint8_t channelIndex);
static uint8_t getPreviousChannelValue(uint8_t encoderState, uint8_t channelIndex);
static inline void initCalibration(Calibration& calibration);
static inline void updateDutyCycleAverage(uint8_t encoderState, unsigned long currentTime);

void initMovingAverage(MovingAverage& buffer) {
  buffer.head = buffer.data;
  buffer.data_end = buffer.data + MOVING_AVERAGE_DATA_SIZE;
  buffer.average = 0;
  buffer.ready = 0;
  for( uint8_t* i = buffer.data; i != buffer.data_end; i++) {
    *i = 0;
  }
}

void addToMovingAverage(MovingAverage& buffer, uint8_t value) {
  uint8_t oldEndValue = *(buffer.head);
  *(buffer.head) = value;

  // NOTE: subtraction first to prevent overflow!
  buffer.average = (buffer.average - (oldEndValue >> 4)) + (value >> 4);
  buffer.head++;
  if( buffer.head == buffer.data_end ) {
    buffer.head = buffer.data;
    buffer.ready = 1;
  }
  // std::cout << "Adding " << (int)value << "\n";
}

uint8_t currentMovingAverage(MovingAverage& buffer) {
  return buffer.average;
}

uint8_t isReadyMovingAverage(MovingAverage& buffer) {
  return buffer.ready;
}

static inline unsigned long calcMicrosDelta(unsigned long currentTime, unsigned long startTime) {
  unsigned long delta;
  // determine how long this measurement has gone on
  if( currentTime < startTime ) {
    // the micros() value rolls over every 70 minutes, correct for that
    delta = currentTime + (ULONG_MAX - startTime);
  } else {
    delta = currentTime - startTime;
  }
  return delta;
}

void initDutyCycleMovingAverage(
  DutyCycleMovingAverage& dutyCycleMA,
  unsigned long currentTimeMicros) {

  initMovingAverage(dutyCycleMA.movingAverage);
  dutyCycleMA.sampleEndTime = currentTimeMicros + DUTY_CYCLE_MA_SAMPLE_LENGTH;
  dutyCycleMA.lastCheckTime = currentTimeMicros;
  dutyCycleMA.acc = 0;
}

static inline uint8_t isDutyCycleGreatherThan50Percent(DutyCycleMovingAverage& dutyCycleMA) {
  return dutyCycleMA.movingAverage.average > 127;
}

void updateDutyCycleMA(
  DutyCycleMovingAverage& dutyCycleMA, 
  uint8_t channelValue, 
  unsigned long currentTimeMicros) {
  //std::cout << "updateDutyCycleMA( " << (int)channelValue << " " << currentTimeMicros << " )\n";
  unsigned long delta = calcMicrosDelta(
    currentTimeMicros, 
    dutyCycleMA.lastCheckTime);

  if( channelValue == HIGH )
    dutyCycleMA.acc += delta;

  if( currentTimeMicros > dutyCycleMA.sampleEndTime ) {
    // trying to avoid multiplication and division here for performance 
    // reasons, but the need branch statement might be negating any
    // performance benefit...

    // the max value of acc is 2^17 (our sample length). We want an 8 bit 
    // number, so just take 8 most significant bits of our 17 bit number.
    if( dutyCycleMA.acc >= DUTY_CYCLE_MA_SAMPLE_LENGTH ) {
      dutyCycleMA.acc = DUTY_CYCLE_MA_SAMPLE_LENGTH-1;
    }
    uint8_t value = (dutyCycleMA.acc >> 9);
    // uint8_t hardValue = (dutyCycleMA.acc/(double)DUTY_CYCLE_MA_SAMPLE_LENGTH)*255;
    // std::cout << (long)dutyCycleMA.acc << " " << (int)value << " " << (int)hardValue << "\n";
    //std::cout << "Preparing to add " << (int)value << " with acc " << dutyCycleMA.acc <<  "\n";
    addToMovingAverage(dutyCycleMA.movingAverage, value);
    dutyCycleMA.sampleEndTime = currentTimeMicros + DUTY_CYCLE_MA_SAMPLE_LENGTH;
    dutyCycleMA.acc = 0;
  }
  dutyCycleMA.lastCheckTime = currentTimeMicros;
}

static inline uint8_t getCurrentChannelValue(uint8_t encoderState, uint8_t channelIndex) {
  return (encoderState >> (5 - channelIndex)) & 0x1;
}

static inline uint8_t getPreviousChannelValue(uint8_t encoderState, uint8_t channelIndex) {
  return (encoderState >> (7 - channelIndex)) & 0x1;
}

static inline void printState(uint8_t encoderState) {
  for( uint8_t i = 0; i < 8; i++ ) {
    if( (encoderState<<i)&0x80 ) {
      Serial.print("0");
    } else {
      Serial.print("1");
    }
  }
  Serial.println("");
}

template<typename POT>
static inline void updateVoltageReference(JTIncrementalEncoder::EncoderChannel& channel, POT& pot) {
  pot.write(channel.channel, map(channel.average, 0, 1023, 0, 255));
}

template<typename STORAGE>
static inline int eepromReadInt(STORAGE& storage, int addr) {
  uint8_t low = storage.read(addr++);
  uint8_t high = storage.read(addr);
  return word(high, low);
}

template<typename STORAGE>
static inline void eepromWriteInt(STORAGE& storage, int addr, int value) {
  uint8_t low = lowByte(value);
  uint8_t high = highByte(value);
  storage.write(addr++, low);
  storage.write(addr, high);
}

template<typename STORAGE>
uint8_t loadSettings(STORAGE& storage, EncoderChannel& channel, int& addr) {
  channel.average = eepromReadInt(storage, addr);
  addr += 2;
  channel.minValue = eepromReadInt(storage, addr);
  addr += 2;
  channel.maxValue = eepromReadInt(storage, addr);
  addr += 2;
  return 0;
}

template<typename STORAGE>
uint8_t saveSettings(STORAGE& storage, const EncoderChannel& channel, int& addr) {
  eepromWriteInt(storage, addr, channel.average);
  addr += 2;
  eepromWriteInt(storage, addr, channel.minValue);
  addr += 2;
  eepromWriteInt(storage, addr, channel.maxValue);
  addr += 2;
  return true;
}

template <typename POT, typename STORAGE, typename LOG>
Encoder<POT,STORAGE, LOG>::Encoder(
    POT& pot,
    STORAGE& EEPROM,
    LOG& log,
    uint8_t channelAInputPin, 
    uint8_t channelARawInputPin,
    MCP42xxx::Channel channelAPotChannel,
    uint8_t channelAInterrupt,
    void (*channelAIsrFunc)(void),
    uint8_t channelBInputPin,
    uint8_t channelBRawInputPin,
    MCP42xxx::Channel channelBPotChannel,
    uint8_t channelBInterrupt,
    void (*channelBIsrFunc)(void)) : pot(pot), storage(EEPROM), log(log) { 

  this->calibrating = 0;
  this->revolutions = 0;

  this->state.channels.a.inputPin = channelAInputPin;
  this->state.channels.a.rawInputPin = channelARawInputPin;
  this->state.channels.a.channel = channelAPotChannel;
  this->state.channels.a.interrupt = channelAInterrupt;
  this->state.channels.a.isrFunc = channelAIsrFunc;
  
  this->state.channels.b.inputPin = channelBInputPin;
  this->state.channels.b.rawInputPin = channelBRawInputPin;
  this->state.channels.b.channel = channelBPotChannel;
  this->state.channels.b.interrupt = channelBInterrupt;
  this->state.channels.b.isrFunc = channelBIsrFunc;  

  pinMode(this->state.channels.a.inputPin, INPUT);
  pinMode(this->state.channels.a.rawInputPin, INPUT);
  pinMode(this->state.channels.b.inputPin, INPUT);
  pinMode(this->state.channels.b.rawInputPin, INPUT);

  // make this for debugging only
  pinMode(INDEX_INDICATOR_PIN, OUTPUT);
}

template <typename POT, typename STORAGE, typename LOG>
void Encoder<POT,STORAGE, LOG>::updateVoltageReference() {
  JTIncrementalEncoder::updateVoltageReference<POT>(this->state.channels.a, this->pot);
  JTIncrementalEncoder::updateVoltageReference<POT>(this->state.channels.b, this->pot);
}

template <typename POT, typename STORAGE, typename LOG>
uint8_t Encoder<POT,STORAGE, LOG>::getDirection(uint8_t encoderState) {
  return JTIncrementalEncoder::getDirection(encoderState);
}

template <typename POT, typename STORAGE, typename LOG>
const EncoderState& Encoder<POT,STORAGE, LOG>::getState() const { return this->state; }

template <typename POT, typename STORAGE, typename LOG>
void Encoder<POT,STORAGE, LOG>::resetPosition() {
  this->state.position = 0;
  this->revolutions = 0;
}

template <typename POT, typename STORAGE, typename LOG>
void Encoder<POT,STORAGE, LOG>::startCalibration() { 
  this->calibrating = 1;
  initCalibration(this->calibration, micros());
  this->pot.write(this->state.channels.a.channel, calibration.a.potSetting);
  this->pot.write(this->state.channels.b.channel, calibration.b.potSetting);
}

template <typename POT, typename STORAGE, typename LOG>
void Encoder<POT,STORAGE, LOG>::stopCalibration() {
  this->calibrating = 0;
}

template <typename POT, typename STORAGE, typename LOG>
void Encoder<POT,STORAGE, LOG>::update() { 
  static uint8_t previousState = 0;
  static uint8_t previousDirection = CLOCKWISE;

  if( this->calibrating ) {
    // for the sake of testing, we get the micros here and pass it as an 
    // argument
    unsigned long currentTime = micros();
    updateDutyCycleAverage(this->state.encoderState, currentTime);

    JTIncrementalEncoder::updateCalibration(
      this->state.channels.a.channel,
      this->state.channels.b.channel,
      this->pot,
      this->calibration,
      this->state.encoderState,
      currentTime);
    if( isCalibrationFinished(calibration) ) {
      this->calibrating = 0;
      this->log.println("Calibration finished");
      this->log.print("Pot setting A: ");
      this->log.println((int)this->calibration.a.potSetting);
      this->log.print("Pot setting B: ");
      this->log.println((int)this->calibration.b.potSetting);
    }
  } else {
    uint8_t currentState = this->state.encoderState;
    int positionTemp = this->state.position;
    if( previousState != currentState ) {
      uint8_t currentDirection = this->getDirection(currentState);
      uint8_t revolutionCompleted = updatePosition(previousState, currentState, positionTemp);
      if( revolutionCompleted )
        this->revolutions++;
      this->positionDidChange(revolutionCompleted, revolutions, previousDirection, state.position, currentDirection, positionTemp);

      previousState = currentState;
      this->state.position = positionTemp;
      previousDirection = currentDirection;
    }
  }
}

template <typename POT, typename STORAGE, typename LOG>
uint8_t Encoder<POT,STORAGE, LOG>::saveSettings(STORAGE& storage, int addr) {
  uint8_t rvalue = true;
  storage.write(addr, 179);
  addr++;
  JTIncrementalEncoder::saveSettings(storage, this->state.channels.a, addr);
  JTIncrementalEncoder::saveSettings(storage, this->state.channels.b, addr);
  return rvalue;
}

template <typename POT, typename STORAGE, typename LOG>
uint8_t Encoder<POT,STORAGE, LOG>::loadSettings(STORAGE& storage, int addr) {
  uint8_t rvalue = true;
  // load settings from EEPROM
  if( 179 == storage.read(addr) ) {
    // Okay, we've written here before, w00t!
    addr++;
    JTIncrementalEncoder::loadSettings(storage, this->state.channels.a, addr);
    JTIncrementalEncoder::loadSettings(storage, this->state.channels.b, addr);
  } else {
    rvalue = false;
  }
  return rvalue;
}

// Note: Passing in by reference here is a performance optimization. This function should *not* actually change these 
// values. This really only matters for the 'int' parameters, but is done for all of them for consistency.
template <typename POT, typename STORAGE, typename LOG>
inline void Encoder<POT,STORAGE, LOG>::positionDidChange(
    const uint8_t& revolutionCompleted,
    const uint8_t& revolutions,
    const uint8_t& previousDirection, 
    const int& previousPosition, 
    const uint8_t& currentDirection, 
    const int& currentPosition) {
  
  // determine if direction changed
  if( currentDirection != previousDirection ) {
    //this->log.println("Direction changed");
  }

  // determine if a revolution has been completed.
  if( revolutionCompleted ) {
    this->log.print("Index: "); this->log.println(revolutions);
    digitalWrite(INDEX_INDICATOR_PIN, LOW);
  } else {
    digitalWrite(INDEX_INDICATOR_PIN, HIGH);
  }
}

/**
 * Given the encoder state, returns the current direction.
 */
static uint8_t getDirection(uint8_t encoderState) {
  uint8_t graycode = encoderState&0xF0; // 11110000
  switch(graycode) {
    case 0xE0: // 11100000
    case 0x80: // 10000000
    case 0x10: // 00010000
    case 0x70: // 01110000
      return CLOCKWISE;
      break;
    default:
      return COUNTERCLOCKWISE;
  }
} 

static inline uint8_t isChannelCalibrationFinished(ChannelCalibration& calibration) {
  return calibration.finished;
}

static inline uint8_t isCalibrationFinished(Calibration& calibration) {
  return calibration.a.finished && calibration.b.finished;
}

static inline void initChannelCalibration(ChannelCalibration& channelCalibration, unsigned long currentTimeMicros) {
  channelCalibration.mask = 0x80;
  channelCalibration.potSetting = 0x80;
  channelCalibration.finished = 0;
  channelCalibration.lastUpdateTime = currentTimeMicros;
  initDutyCycleMovingAverage(channelCalibration.dutyCycleMA, currentTimeMicros);
}

static inline void initCalibration(Calibration& calibration, unsigned long currentTimeMicros) {
  initChannelCalibration(calibration.a, currentTimeMicros);
  initChannelCalibration(calibration.b, currentTimeMicros);
}

static inline void updateChannelCalibration(ChannelCalibration& calibration, uint8_t currentChannelValue, unsigned long currentTime) {
  if( ! calibration.finished ) {
    updateDutyCycleMA(calibration.dutyCycleMA, currentChannelValue, currentTime);

    unsigned long delta = calcMicrosDelta(currentTime, calibration.lastUpdateTime);
    
    if( delta > CALIBRATION_INTERVAL_LENGTH ) {  
      // NOTE: Higher POT settings give lower resistance
      if( isDutyCycleGreatherThan50Percent(calibration.dutyCycleMA) ) {
        // the reference voltage is too low causing us to stay in a HIGH state
        // too long. We raise the voltage by decreasing resistance, which we
        // achieve by increasing the POT setting. So leave this bit set.

      } else {
        // the reference voltage is too high causing us to stay in a LOW state
        // too long. We lower the voltage by increasing resistance, which we
        // achieve by decreasing the POT setting. So unset this bit.
        calibration.potSetting = calibration.potSetting ^ calibration.mask;
      }

      if( calibration.mask == 1 ) {
        // finished calibration
        calibration.finished = 1;
      } else {
        // set next bit of potSetting
        calibration.mask = calibration.mask >> 1;
        calibration.potSetting = calibration.potSetting | calibration.mask;
      }

      // reset the last update time as we are beginning a new interval
      calibration.lastUpdateTime = currentTime;
    }
  }
}


static inline void updateDutyCycleMovingAverage(
  DutyCycleMovingAverage& dutyCycleMA, 
  uint8_t channelValue, 
  unsigned long currentTimeMicros) {

  updateDutyCycleMA(dutyCycleMA, channelValue, currentTimeMicros);
}

template<typename POT>
static inline void updateCalibration(
  MCP42xxx::Channel& channelA, 
  MCP42xxx::Channel& channelB, 
  POT& pot, 
  Calibration& calibration, 
  uint8_t encoderState, 
  unsigned long currentTime) {

  uint8_t channelAValue = getCurrentChannelValue(encoderState, 0);
  uint8_t channelBValue = getCurrentChannelValue(encoderState, 1);
  uint8_t oldChannelAPotSetting = calibration.a.potSetting;
  uint8_t oldChannelBPotSetting = calibration.b.potSetting;

  updateChannelCalibration(calibration.a, channelAValue, currentTime);
  updateChannelCalibration(calibration.b, channelBValue, currentTime);

  if( oldChannelAPotSetting != calibration.a.potSetting ) {
    pot.write(channelA, calibration.a.potSetting);
  }

  if( oldChannelBPotSetting != calibration.b.potSetting ) {
    pot.write(channelB, calibration.b.potSetting);  
  }
}

static inline uint8_t updatePosition(uint8_t previousState, uint8_t currentState, int &position) {
  uint8_t currentDirection = getDirection(currentState);
  uint8_t currentStepIndex = (currentState&0x0F/*B00001111*/) + 1;
  uint8_t previousStepIndex = (previousState&0x0F/*B00001111*/) + 1;
  uint8_t delta;

  if( currentStepIndex > previousStepIndex ) {
    delta = currentStepIndex - previousStepIndex;
  } else {
    delta = 16 - previousStepIndex + currentStepIndex;
  }
  
  // update position
  if( currentDirection == CLOCKWISE ) {
    position += delta;
  } else {
    position -= delta;
  }

  // determine if we completed a revolution and fix the position if neccessary
  uint8_t revolutionCompleted = 0;
  if( position >= DIVISIONS ) {
    revolutionCompleted = 1;
    position -= DIVISIONS;
  } else if( position <= 0 ) {
    revolutionCompleted = 1;
    position += DIVISIONS;
  }

  return revolutionCompleted;
}

#ifndef JTIncrementalEncoder_SKIP_DEFAULT_TEMPLATE_INSTANTIATION
template class Encoder<MCP42xxx,EEPROMClass,Print>;
#endif

}

#include <limits.h>
#include "encoder.h"

namespace JTIncrementalEncoder {

static const int INDEX_INDICATOR_PIN = 7;

static inline void stopCalibration(EncoderChannelPair& channelPair);
static uint8_t getDirection(uint8_t encoderState);
static uint8_t updatePosition(uint8_t previousState, uint8_t currentState, int &position);
static uint8_t getCurrentChannelValue(uint8_t encoderState, uint8_t channelIndex);
static uint8_t getPreviousChannelValue(uint8_t encoderState, uint8_t channelIndex);

template<typename CALIBRATOR>
static inline uint8_t isCalibrationFinished(Calibration<CALIBRATOR>& calibration);

template<typename CALIBRATOR>
static inline void updateCalibration(
  Calibration<CALIBRATOR>& calibration, 
  unsigned long currentTime);

static inline uint8_t getCurrentChannelValue(uint8_t encoderState, uint8_t channelIndex) {
  return (encoderState >> (5 - channelIndex)) & 0x1;
}

static inline uint8_t getPreviousChannelValue(uint8_t encoderState, uint8_t channelIndex) {
  return (encoderState >> (7 - channelIndex)) & 0x1;
}

static inline void printState(uint8_t encoderState) {
  for( uint8_t i = 0; i < 8; i++ ) {
    if( (encoderState<<i)&0x80 ) {
      Serial.print("1");
    } else {
      Serial.print("0"); 
    }
  }
//  Serial.println("");
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

template <typename CALIBRATOR, typename STORAGE, typename LOG>
Encoder<CALIBRATOR, STORAGE, LOG>::Encoder(
    STORAGE& EEPROM,
    LOG& log,
    Calibration<CALIBRATOR>& calibration,
    uint8_t channelAInputPin,
    uint8_t channelAInterrupt,
    void (*channelAIsrFunc)(void),
    uint8_t channelBInputPin,
    uint8_t channelBInterrupt,
    void (*channelBIsrFunc)(void)) : calibration(calibration), storage(EEPROM), log(log) { 

  this->calibrating = 0;
  this->revolutions = 0;

  this->state.channels.a.inputPin = channelAInputPin;
  this->state.channels.a.interrupt = channelAInterrupt;
  this->state.channels.a.isrFunc = channelAIsrFunc;
  
  this->state.channels.b.inputPin = channelBInputPin;
  this->state.channels.b.interrupt = channelBInterrupt;
  this->state.channels.b.isrFunc = channelBIsrFunc;  

  pinMode(this->state.channels.a.inputPin, INPUT);
  pinMode(this->state.channels.b.inputPin, INPUT);

  // make this for debugging only
  pinMode(INDEX_INDICATOR_PIN, OUTPUT);

  attachInterrupt(this->state.channels.a.interrupt, this->state.channels.a.isrFunc, CHANGE);
  attachInterrupt(this->state.channels.b.interrupt, this->state.channels.b.isrFunc, CHANGE);
}

template <typename CALIBRATOR, typename STORAGE, typename LOG>
uint8_t Encoder<CALIBRATOR, STORAGE, LOG>::getDirection(uint8_t encoderState) {
  return JTIncrementalEncoder::getDirection(encoderState);
}

template <typename CALIBRATOR, typename STORAGE, typename LOG>
const EncoderState& Encoder<CALIBRATOR, STORAGE, LOG>::getState() const { return this->state; }

template <typename CALIBRATOR, typename STORAGE, typename LOG>
void Encoder<CALIBRATOR, STORAGE, LOG>::resetPosition() {
  this->state.position = 0;
  this->revolutions = 0;
}

template <typename CALIBRATOR, typename STORAGE, typename LOG>
void Encoder<CALIBRATOR, STORAGE, LOG>::startCalibration() { 
  if( !this->calibrating ) {
    unsigned long currentTime = millis();
    this->state.calibrated = 0;
    this->calibrating = 1;
    this->calibration.a.begin(currentTime);
    this->calibration.b.begin(currentTime);
  }
}

template <typename CALIBRATOR, typename STORAGE, typename LOG>
void Encoder<CALIBRATOR, STORAGE, LOG>::stopCalibration() {
  //this->calibrating = 0;
}

template <typename CALIBRATOR, typename STORAGE, typename LOG>
void Encoder<CALIBRATOR, STORAGE, LOG>::update() { 
  static uint8_t previousState = 0;
  static uint8_t previousDirection = CLOCKWISE;
  uint8_t currentState = this->state.encoderState;
  if( previousState != currentState ) {
    //printState(currentState);
  }
  
  if( this->calibrating ) {
    unsigned long currentTime = millis();
    JTIncrementalEncoder::updateCalibration(
      this->calibration,
      currentTime);
    if( isCalibrationFinished(calibration) ) {
      this->calibrating = 0;
      this->state.calibrated = 1;
      // calibration takes over the interrupts, restore them now that
      // calibration is done
      attachInterrupt(this->state.channels.a.interrupt, this->state.channels.a.isrFunc, CHANGE);
      attachInterrupt(this->state.channels.b.interrupt, this->state.channels.b.isrFunc, CHANGE);
    }
  } else {
    //Serial.print("Raw: "); Serial.println(analogRead(this->state.channels.a.rawInputPin)); //Serial.print(" "); Serial.println(analogRead(this->state.channels.b.rawInputPin));
    int positionTemp = this->state.position;
    if( previousState != currentState ) {
      uint8_t currentDirection = this->getDirection(currentState);
      uint8_t revolutionCompleted = updatePosition(previousState, currentState, positionTemp);
      if( revolutionCompleted )
        this->revolutions++;
      this->positionDidChange(revolutionCompleted, revolutions, previousDirection, state.position, currentDirection, positionTemp);

      
      this->state.position = positionTemp;
      previousDirection = currentDirection;
    }
  }
  previousState = currentState;
}

template <typename CALIBRATOR, typename STORAGE, typename LOG>
uint8_t Encoder<CALIBRATOR, STORAGE, LOG>::saveSettings(STORAGE& storage, int addr) {
  uint8_t rvalue = true;
  storage.write(addr, 179);
  addr++;
  JTIncrementalEncoder::saveSettings(storage, this->state.channels.a, addr);
  JTIncrementalEncoder::saveSettings(storage, this->state.channels.b, addr);
  return rvalue;
}

template <typename CALIBRATOR, typename STORAGE, typename LOG>
uint8_t Encoder<CALIBRATOR, STORAGE, LOG>::loadSettings(STORAGE& storage, int addr) {
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
template <typename CALIBRATOR, typename STORAGE, typename LOG>
inline void Encoder<CALIBRATOR, STORAGE, LOG>::positionDidChange(
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
    // this->log.print("Index: "); this->log.println(revolutions);
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

template<typename CALIBRATOR>
static inline uint8_t isCalibrationFinished(Calibration<CALIBRATOR>& calibration) {
  return calibration.a.isFinished() && calibration.b.isFinished();
}

template<typename CALIBRATOR>
static inline void updateCalibration(
  Calibration<CALIBRATOR>& calibration, 
  unsigned long currentTime) {

  calibration.a.update(currentTime);
  calibration.b.update(currentTime);
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
#include <EEPROM.h>
template class Encoder<DefaultReferenceCalibrator,EEPROMClass,Print>;
#endif

}


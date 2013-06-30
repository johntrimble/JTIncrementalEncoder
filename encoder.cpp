#include <EEPROM.h>
#include "encoder.h"

namespace JTIncrementalEncoder {

static const int INDEX_INDICATOR_PIN = 7;

static void startCalibration(EncoderChannelPair& channelPair);
static void updateCalibration(EncoderChannelPair& channelPair);
static void stopCalibration(EncoderChannelPair& channelPair);
static uint8_t getDirection(uint8_t encoderState);
static uint8_t updatePosition(uint8_t previousState, uint8_t currentState, int &position);

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
const EncoderState& Encoder<POT,STORAGE, LOG>::getState() { return this->state; }

template <typename POT, typename STORAGE, typename LOG>
void Encoder<POT,STORAGE, LOG>::resetPosition() {
  this->state.position = 0;
  this->revolutions = 0;
}

template <typename POT, typename STORAGE, typename LOG>
void Encoder<POT,STORAGE, LOG>::startCalibration() { 
  this->calibrating = 1;
  JTIncrementalEncoder::startCalibration(this->state.channels);
}

template <typename POT, typename STORAGE, typename LOG>
void Encoder<POT,STORAGE, LOG>::stopCalibration() { 
  JTIncrementalEncoder::stopCalibration(this->state.channels);
  this->updateVoltageReference();
  this->calibrating = 0;
}

template <typename POT, typename STORAGE, typename LOG>
void Encoder<POT,STORAGE, LOG>::update() { 
  static uint8_t previousState = 0;
  static uint8_t previousDirection = CLOCKWISE;

  if( this->calibrating ) {
    JTIncrementalEncoder::updateCalibration(this->state.channels);
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
// values. This really only matters for the 'int' parameters, but is done for all of the for consistency.
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
    this->log.println("Direction changed");
  }

  // determine if a revolution has been completed.
  if( revolutionCompleted ) {
    this->log.print("Index: "); this->log.println(revolutions);
    digitalWrite(INDEX_INDICATOR_PIN, LOW);
  } else {
    digitalWrite(INDEX_INDICATOR_PIN, HIGH);
  }
}

void isort(int arr[], int length) {
  for( int i = 1; i < length; i++ ) {
    int value = arr[i];
    int k = i;
    while( k > 0 && value < arr[k-1] ) {
       arr[k] = arr[k-1];
       k--;      
    }
    arr[k] = value;
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

/**
 * Reads from the given analog pin multiple times and returns the mean.
 */
uint8_t sample(uint8_t pin) {
  int readings[5];
  for( int i = 0; i < 5; i++ ) {
    readings[i] = analogRead(pin);
  }
  isort(readings, 5);
  return readings[2];
}

static inline void startCalibration(EncoderChannelPair& channelPair) {
  detachInterrupt(0);
  detachInterrupt(1);

  channelPair.a.average = 0;
  channelPair.a.minValue = 1023;
  channelPair.a.maxValue = 0;
  channelPair.b.average = 0;
  channelPair.b.minValue = 1023;
  channelPair.b.maxValue = 0;
}

static inline void updateCalibration(EncoderChannelPair& channelPair) {
  EncoderChannel *channels = &(channelPair.a);
  for( int i = 0; i < NUMBER_CHANNELS; i++ ) {
    int value = sample(channels[i].rawInputPin);
    channels[i].minValue = min(channels[i].minValue, value);
    channels[i].maxValue = max(channels[i].maxValue, value);
  }
}

static inline void stopCalibration(EncoderChannelPair& channelPair) {
  EncoderChannel *channels = &(channelPair.a);
  // set averages and adjust digital potentiometers accordingly
  for( int i = 0; i < NUMBER_CHANNELS; i++ ) {
    //channels[i].average = total[i] / count;
    channels[i].average = (channels[i].minValue + channels[i].maxValue) / 2;
  }
  attachInterrupt(channelPair.a.interrupt, channelPair.a.isrFunc, CHANGE);
  attachInterrupt(channelPair.b.interrupt, channelPair.b.isrFunc, CHANGE);
}

static uint8_t updatePosition(uint8_t previousState, uint8_t currentState, int &position) {
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
  
template class Encoder<MCP42xxx,EEPROMClass,Print>;

}

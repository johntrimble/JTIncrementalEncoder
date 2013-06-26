#include <EEPROM.h>
#include "encoder.h"


namespace JTIncrementalEncoder {

static int eepromReadInt(int addr);
static void eepromWriteInt(int addr, int value);
static uint8_t loadSettings(EncoderChannel& channel, int& addr);
static uint8_t saveSettings(const EncoderChannel& channel, int& addr);


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

static int eepromReadInt(int addr) {
  uint8_t low = EEPROM.read(addr++);
  uint8_t high = EEPROM.read(addr);
  return word(high, low);
}

static void eepromWriteInt(int addr, int value) {
  uint8_t low = lowByte(value);
  uint8_t high = highByte(value);
  EEPROM.write(addr++, low);
  EEPROM.write(addr, high);
}

uint8_t loadSettings(EncoderChannel& channel, int& addr) {
  channel.average = eepromReadInt(addr);
  addr += 2;
  channel.minValue = eepromReadInt(addr);
  addr += 2;
  channel.maxValue = eepromReadInt(addr);
  addr += 2;
}

uint8_t saveSettings(const EncoderChannel& channel, int& addr) {
  eepromWriteInt(addr, channel.average);
  addr += 2;
  eepromWriteInt(addr, channel.minValue);
  addr += 2;
  eepromWriteInt(addr, channel.maxValue);
  addr += 2;
  return true;
}

uint8_t loadSettings(int addr, EncoderChannelPair& channels) {
  uint8_t rvalue = true;
  // load settings from EEPROM
  if( 179 == EEPROM.read(addr) ) {
    // Okay, we've written here before, w00t!
    addr++;
    loadSettings(channels.a, addr);
    loadSettings(channels.b, addr);
  } else {
    rvalue = false;
  }
  return rvalue;
}

uint8_t saveSettings(int addr, const EncoderChannelPair& channels) {
  uint8_t rvalue = true;
  EEPROM.write(addr, 179);
  addr++;
  saveSettings(channels.a, addr);
  saveSettings(channels.b, addr);
  return rvalue;
}

/**
 * Given the encoder state, returns the current direction.
 */
uint8_t getDirection(uint8_t encoderState) {
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

void runCalibration(EncoderChannelPair& channelPair) {
  // this is a cheat so that I can iterate over the encoders as if they were in an array.
  EncoderChannel *channels = &(channelPair.a);

  // set default values
  long total[NUMBER_CHANNELS];
  for(int i = 0; i < NUMBER_CHANNELS; i++ ) {
    total[i] = 0;
    channels[i].average = 0;
    channels[i].minValue = 1023;
    channels[i].maxValue = 0;
  }
  
  // collect readings
  long count = 0;
  while( EncoderInterface.getCalibrationMode() ) {
    for( int i = 0; i < NUMBER_CHANNELS; i++ ) {
      int value = sample(channels[i].rawInputPin);
      channels[i].minValue = min(channels[i].minValue, value);
      channels[i].maxValue = max(channels[i].maxValue, value);
      total[i] += value;
      count++;
    }
  }
  
  // set averages and adjust digital potentiometers accordingly
  for( int i = 0; i < NUMBER_CHANNELS; i++ ) {
    //channels[i].average = total[i] / count;
    channels[i].average = (channels[i].minValue + channels[i].maxValue) / 2;
  }
}

uint8_t updatePosition(uint8_t previousState, uint8_t currentState, int &position) {
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
  
}

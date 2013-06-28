#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include <MCP42xxx.h>
#include "JTIncrementalEncoder.h"
#include "encoder.h"
#include "interface.h"

#define log_println(X) Serial.println((X))
#define log_print(X) Serial.print((X))

using namespace JTIncrementalEncoder;

// General encoder constants
static const int SLAVE_ADDRESS = 31;
static const int CALIBRATION_PIN = 6;
static const int RESET_PIN = 8;
static const int INDEX_INDICATOR_PIN = 7;

// Channel A
static const int CHANNEL_A_INPUT_PIN = 2;
static const int CHANNEL_A_INTERRUPT = 0;
static const int CHANNEL_A_RAW_INPUT_PIN = A0;

// Channel B
static const int CHANNEL_B_INPUT_PIN = 3;
static const int CHANNEL_B_INTERRUPT = 1;
static const int CHANNEL_B_RAW_INPUT_PIN = A1;

// Interrupt vars
static byte isrSteps = 0;

static EncoderState state = {};

static EncoderChannelPair& channels = state.channels;

MCP42xxx pot(10, -1, -1);
Encoder encoder(
  pot, 
  EEPROM,
  Serial,
  CHANNEL_A_INPUT_PIN, 
  CHANNEL_A_RAW_INPUT_PIN,
  MCP42xxx::CHANNEL_0,
  CHANNEL_A_INTERRUPT,
  &channelAISR,
  CHANNEL_B_INPUT_PIN,
  CHANNEL_B_RAW_INPUT_PIN,
  MCP42xxx::CHANNEL_1,
  CHANNEL_B_INTERRUPT,
  &channelBISR);

////////////////////////////////////////////////////////
// Things that deal with channels, position, etc.
////////////////////////////////////////////////////////

/**
 * Interrupt handlers for channel A and B. Updates encoderState.
 */
static void channelAISR() {
  updateChannelAEncoderState(channels.a, state.encoderState, isrSteps);
}

static void channelBISR() {
  updateChannelBEncoderState(channels.b, state.encoderState, isrSteps);
}

static void printChannelInfo(EncoderChannel& channel) {
  Serial.print("Average: "); Serial.println(channel.average);
  Serial.print("Min: "); Serial.println(channel.minValue);
  Serial.print("Max: "); Serial.println(channel.maxValue);
}

static void updateVoltageReference(JTIncrementalEncoder::EncoderChannel& channel) {
  pot.write(channel.channel, map(channel.average, 0, 1023, 0, 255));
}

static void updateVoltageReferences() {
  Serial.println("Updating potentiometers");
  updateVoltageReference(channels.a);
  updateVoltageReference(channels.b);
}

static void initializeChannels() {
  channels.a.inputPin = CHANNEL_A_INPUT_PIN;
  channels.a.rawInputPin = CHANNEL_A_RAW_INPUT_PIN;
  channels.a.channel = MCP42xxx::CHANNEL_0;
  channels.a.interrupt = CHANNEL_A_INTERRUPT;
  channels.a.isrFunc = &channelAISR;
  
  channels.b.inputPin = CHANNEL_B_INPUT_PIN;
  channels.b.rawInputPin = CHANNEL_B_RAW_INPUT_PIN;
  channels.b.channel = MCP42xxx::CHANNEL_1;
  channels.b.interrupt = CHANNEL_B_INTERRUPT;
  channels.b.isrFunc = &channelBISR;  

  pinMode(channels.a.inputPin, INPUT);
  pinMode(channels.a.rawInputPin, INPUT);
  pinMode(channels.b.inputPin, INPUT);
  pinMode(channels.b.rawInputPin, INPUT);
}

// Note: Passing in by reference here is a performance optimization. This function should *not* actually change these 
// values. This really only matters for the 'int' parameters, but is done for all of the for consistency.
static inline void positionDidChange(
    const uint8_t& revolutionCompleted,
    const uint8_t& revolutions,
    const uint8_t& previousDirection, 
    const int& previousPosition, 
    const uint8_t& currentDirection, 
    const int& currentPosition) {
  
  // determine if direction changed
  if( currentDirection != previousDirection ) {
    Serial.println("Direction changed");
  }

  // determine if a revolution has been completed.
  if( revolutionCompleted ) {
    Serial.print("Index: "); Serial.println(revolutions);
    digitalWrite(INDEX_INDICATOR_PIN, LOW);
  } else {
    digitalWrite(INDEX_INDICATOR_PIN, HIGH);
  }
}

void setup() {
  SPI.begin();
  Serial.begin(9600);
  
  initializeChannels();
  
  // set up calibration and reset pin
  //pinMode(CALIBRATION_PIN, INPUT);
  pinMode(RESET_PIN, INPUT);
  pinMode(INDEX_INDICATOR_PIN, OUTPUT);
  digitalWrite(INDEX_INDICATOR_PIN, HIGH);
  
  // load settings from EEPROM
  state.calibrated = loadSettings(0, channels);
  if( state.calibrated ) {
    updateVoltageReferences();
  }
  
  EncoderInterface.update(state);
  EncoderInterface.begin(SLAVE_ADDRESS);
}

void loop() {
  static byte previousState = 0;
  static byte previousDirection = CLOCKWISE;
  static byte revolutions = 0;
  static uint8_t calibrating = 0;
  
  uint8_t newCalibratingValue = EncoderInterface.getCalibrationMode();

  if( !calibrating && newCalibratingValue ) {
    Serial.println("Calibrating");
    startCalibration(channels);
  } else if( calibrating && !newCalibratingValue ) {
    stopCalibration(channels);
    saveSettings(0, channels);
    printChannelInfo(channels.a);
    printChannelInfo(channels.b);
    updateVoltageReferences();
  }

  calibrating = newCalibratingValue;

  if( calibrating ) {
    updateCalibrating(channels);
  } else {
    int positionTemp = state.position;

    if( EncoderInterface.getResetHomeMode() ) {
      Serial.println("Reset");
      positionTemp = 0;
      state.position = 0;
      revolutions = 0;
      EncoderInterface.setResetHomeMode(false);
    }

    // handle state change
    byte currentState = state.encoderState;
    if( previousState != currentState ) {
      byte currentDirection = getDirection(currentState);
      
      // update positionTemp
      byte revolutionCompleted = updatePosition(previousState, currentState, positionTemp);
      if( revolutionCompleted )
        revolutions++;
      positionDidChange(revolutionCompleted, revolutions, previousDirection, state.position, currentDirection, positionTemp);
      
      // update state
      previousState = currentState;
      state.position = positionTemp;
      previousDirection = currentDirection;
      
    } // end handle state change
  }
  EncoderInterface.update(state);
}




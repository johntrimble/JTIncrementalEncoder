#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include <MCP42xxx.h>
#include "JTIncrementalEncoder.h"
#include "encoder.h"
#include "interface.h"

using namespace JTIncrementalEncoder;

// General encoder constants
static const int SLAVE_ADDRESS = 31;

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

// build and configure object graph
MCP42xxx pot(10, -1, -1);
Encoder<MCP42xxx,EEPROMClass,Print> encoder(
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
EncoderInterface<TwoWire, Encoder<MCP42xxx,EEPROMClass,Print> > i2cSlave(Wire, encoder);

/**
 * Interrupt handlers for channel A and B. Updates encoderState.
 * TODO: The Encoder class does not define the interrupt handlers, but does add them as interrupt handlers. Either it
 * should both define and add the interrupts or have no knowledge of interrupts at all, but not both.
 */
static void channelAISR() {
  updateChannelAEncoderState(encoder.state.channels.a, encoder.state.encoderState, isrSteps);
}

static void channelBISR() {
  updateChannelBEncoderState(encoder.state.channels.b, encoder.state.encoderState, isrSteps);
}

void setup() {
  SPI.begin();
  Serial.begin(9600);
  encoder.loadSettings(EEPROM, 0);
  i2cSlave.begin(SLAVE_ADDRESS);
}

void loop() {
  static uint8_t calibratedStatus = i2cSlave.getCalibratedStatus();
  i2cSlave.update();
  static uint8_t newCalibratedStatus = i2cSlave.getCalibratedStatus();
  if( newCalibratedStatus && newCalibratedStatus != calibratedStatus ) {
    // we've recalibrated, save settings
    // TODO: This seems like a sloppy way to figure out if we should save. Find a way to remove the suck from this.
    encoder.saveSettings(EEPROM, 0);
  }
  calibratedStatus = newCalibratedStatus; 
}

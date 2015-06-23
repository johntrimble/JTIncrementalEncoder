#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include "JTIncrementalEncoder.h"
#include "encoder.h"
#include "interface.h"
#include "jt_calibrator.h"

using namespace JTIncrementalEncoder;

// General encoder constants
static const int SLAVE_ADDRESS = 31;

// Channel A
static const int CHANNEL_A_INPUT_PIN = 2;
static const int CHANNEL_A_INTERRUPT = 0;
static const int CHANNEL_A_REFERENCE_PIN = 9;

// Channel B
static const int CHANNEL_B_INPUT_PIN = 3;
static const int CHANNEL_B_INTERRUPT = 1;
static const int CHANNEL_B_REFERENCE_PIN = 10;

// Interrupt vars
static byte isrSteps = 0;

// build and configure the object graph
DigitalInputPin inputA(CHANNEL_A_INPUT_PIN);
AnalogOutputPin referenceA(CHANNEL_A_REFERENCE_PIN);
DefaultReferenceCalibrator calibratorA(
  referenceA, 
  interrupt0, 
  inputA, 
  Serial, 
  1000, 
  1000);

DigitalInputPin inputB(CHANNEL_B_INPUT_PIN);
AnalogOutputPin referenceB(CHANNEL_B_REFERENCE_PIN);
DefaultReferenceCalibrator calibratorB(
  referenceB, 
  interrupt1, 
  inputB, 
  Serial, 
  1000, 
  1000);

Calibration calibration = {calibratorA, calibratorB};

Encoder<DefaultReferenceCalibrator, EEPROMClass,Print> encoder(
  EEPROM,
  Serial,
  calibration,
  CHANNEL_A_INPUT_PIN,
  CHANNEL_A_INTERRUPT,
  &channelAISR,
  CHANNEL_B_INPUT_PIN,
  CHANNEL_B_INTERRUPT,
  &channelBISR);

EncoderInterface<TwoWire, Encoder<EEPROMClass,Print> > i2cSlave(Wire, encoder);

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

void setupTimer1() {
  noInterrupts();
  // set timer 1 to 7812.5Hz
  // 16MHz / (prescaler * (1 + TOP))
  // 16MHz / (8 * 256) = 7812.5Hz
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  // fast PWM with TOP value of 255 (0xFF)
  TCCR1A |= (1<<WGM10);
  TCCR1B |= (1<<WGM12);
  // non-inverting mode
  TCCR1A |= (1<<COM1A1)|(1<<COM1B1);
  // prescaler of 8
  TCCR1B |= (1<<CS11);
  interrupts();
}

void setup() {
  setupTimer1();
  Serial.begin(9600);
  //encoder.loadSettings(EEPROM, 0);
  i2cSlave.begin(SLAVE_ADDRESS);
}

void loop() {
  static uint8_t calibratedStatus = i2cSlave.getCalibratedStatus();
  i2cSlave.update();
  static uint8_t newCalibratedStatus = i2cSlave.getCalibratedStatus();
  if( newCalibratedStatus && newCalibratedStatus != calibratedStatus ) {
    // we've recalibrated, save settings
    // TODO: This seems like a sloppy way to figure out if we should save. Find a way to remove the suck from this.
    //encoder.saveSettings(EEPROM, 0);
  }
  calibratedStatus = newCalibratedStatus; 
}

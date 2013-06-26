#include <Wire.h>
#include "interface.h"

namespace JTIncrementalEncoder {

/* I2C interface
 * Control register...
 * Status: Ready, Busy
 * Error: ...
 * State: ...
 * Position: ...
 * Modes: Calibrating, Reset Home Position, Tracking
 * Divisions: ...
 * Identificaion: ...
 */


// Configuration register
// D7 D6 D5 D4 D3 D2 D1 D0 
// X  X  X  X  X  CM RH TM
//
// CM - Calibration Mode (1 means calibrating)
// RH - Reset Home (1 means resetting home position)
// TM - Tracking Mode (1 means currently tracking)
static const uint8_t CALIBRATION_MODE = B00000100;
static const uint8_t RESET_HOME_MODE  = B00000010;
static const uint8_t TRACKING_MODE    = B00000001;

static const uint8_t READY_STATUS        = 0;
static const uint8_t BUSY_STATUS         = B00000010;
static const uint8_t UNCALIBRATED_STATUS = B00000001;

static const uint8_t STATUS_REGISTER = 0;
static const uint8_t ERROR_REGISTER = 1;
static const uint8_t STATE_REGISTER = 2;
static const uint8_t POSITION_REGISTER = 3;
static const uint8_t MODE_INDEX = 4;
static const uint8_t DIVISION_REGISTER = 5;
static const uint8_t IDENTIFICATION_REGISTER = 6;

static void requestEvent();
static void receiveEvent(int bytesReceived);

EncoderInterface::EncoderInterface(TwoWire& twoWire) : twoWire(twoWire) {
  this->registers[STATUS_REGISTER] = READY_STATUS;
  this->registers[ERROR_REGISTER] = 0;
  this->registers[STATE_REGISTER] = 0;
  this->registers[POSITION_REGISTER] = 0;
  this->registers[MODE_INDEX] = TRACKING_MODE;
  this->registers[DIVISION_REGISTER] = DIVISIONS;
  this->registers[IDENTIFICATION_REGISTER] = 0;
}

void EncoderInterface::begin(uint8_t slaveAddress) {
  this->twoWire.begin(slaveAddress);
  this->twoWire.onReceive(&JTIncrementalEncoder::receiveEvent);
  this->twoWire.onRequest(&JTIncrementalEncoder::requestEvent);
}

/**
 * Updates the registers based on what the master wrote to this device. We don't do this in the
 * I2C interrupt handler as it is generally best practice to minimize the amount of time spent
 * in an interrupt handler.
 */
void EncoderInterface::update(EncoderState& state) {
  this->update();

  if( state.calibrated ) {
    this->registers[STATUS_REGISTER] &= ~UNCALIBRATED_STATUS;
  } else {
    this->registers[STATUS_REGISTER] |= UNCALIBRATED_STATUS;
  }
  
  this->registers[POSITION_REGISTER] = state.position;
  this->registers[STATE_REGISTER] = state.encoderState;
}

void EncoderInterface::update() {
  if( this->bytesReceivedFromMaster ) {
    int offset = this->receivedData[0];
    for(int i = 1; i < this->bytesReceivedFromMaster; i++ ) {
      int registerIndex = offset + i - 1;
      uint8_t temp = this->receivedData[i];
      if( registerIndex > 3 && registerIndex < 6 ) { // only registers 4 and 5 are writable
        if( MODE_INDEX == registerIndex ) {
          if( !(temp&(CALIBRATION_MODE|RESET_HOME_MODE|TRACKING_MODE)) ) {
            // MODE was not set to a valid value, so just set it to tracking
            temp = TRACKING_MODE;
          }
        }
        this->registers[registerIndex] = temp;
      }
    }
    
    this->registers[STATUS_REGISTER] &= ~BUSY_STATUS;
    this->bytesReceivedFromMaster = 0;
  }
}

uint8_t EncoderInterface::getCalibrationMode() {
  this->update(); // check if calibration mode changed via I2C
  return this->registers[MODE_INDEX]&CALIBRATION_MODE;
}

void EncoderInterface::setCalibrationMode(uint8_t val) {
  this->update();
  if( val )
    this->registers[MODE_INDEX] |= CALIBRATION_MODE;
  else
    this->registers[MODE_INDEX] &= ~CALIBRATION_MODE;
}

uint8_t EncoderInterface::getResetHomeMode() {
  this->update();
  return this->registers[MODE_INDEX]&RESET_HOME_MODE;
}

void EncoderInterface::setResetHomeMode(uint8_t val) {
  this->update();
  if( val ) {
    this->registers[MODE_INDEX] |= RESET_HOME_MODE;
  } else {
    this->registers[MODE_INDEX] &= ~RESET_HOME_MODE;
  }
}

uint8_t EncoderInterface::getCalibratedStatus() {
  return this->registers[STATUS_REGISTER] & UNCALIBRATED_STATUS ? 1 : 0;
}

void EncoderInterface::setCalibratedStatus(uint8_t val) {
  if( val ) {
    this->registers[STATUS_REGISTER] &= ~UNCALIBRATED_STATUS;
  } else {
    this->registers[STATUS_REGISTER] |= UNCALIBRATED_STATUS;
  }
}

void EncoderInterface::requestEvent() {
  this->twoWire.write(this->registers + this->receivedData[0], NUMBER_REGISTERS - this->receivedData[0]);
}

void EncoderInterface::receiveEvent(int bytesReceived) {
  this->registers[STATUS_REGISTER] |= BUSY_STATUS;
  for( byte i = 0; i < bytesReceived; i++ ) {
    if( i < MAX_WRITE_BYTES ) {
      this->receivedData[i] = this->twoWire.read();
    } else {
      this->twoWire.read();
    }
  }
  this->bytesReceivedFromMaster = bytesReceived;
}

/**
 * I2C handler for when master is requesting data from this slave.
 */
static void requestEvent() {
  EncoderInterface.requestEvent();
}

/**
 * I2C handler for when master is writing data to this slave. Format is as follows:
 * - First byte is always the register address byte.
 * - Remaining bytes are to be written to registers, starting at the provided address.
 * If more data is provided than will fit in to the write registers, the excess data is sliently
 * dropped.
 */
static void receiveEvent(int bytesReceived) {
  EncoderInterface.receiveEvent(bytesReceived);
}

EncoderInterface::EncoderInterface EncoderInterface(Wire);

}

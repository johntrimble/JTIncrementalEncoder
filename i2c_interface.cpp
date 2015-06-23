
#include "interface.h"

namespace JTIncrementalEncoder {

template <typename WIRE, typename ENCODER>
EncoderInterface<WIRE,ENCODER> *EncoderInterface<WIRE,ENCODER>::singleton = NULL;

template <typename WIRE, typename ENCODER>
EncoderInterface<WIRE,ENCODER>::EncoderInterface(WIRE& twoWire, ENCODER& encoder) : twoWire(twoWire), encoder(encoder) {
  this->registers[STATUS_REGISTER] = READY_STATUS;
  this->registers[ERROR_REGISTER] = 0;
  this->registers[STATE_REGISTER] = 0;
  this->registers[POSITION_REGISTER] = 0;
  this->registers[MODE_INDEX] = TRACKING_MODE;
  this->registers[DIVISION_REGISTER] = DIVISIONS;
  this->registers[IDENTIFICATION_REGISTER] = 0;
  this->bytesReceivedFromMaster = 0;
  for( uint8_t i = 0; i < MAX_WRITE_BYTES; i++ ) {
    this->receivedData[i] = 0;
  }
}

/**
 * Calls begin(..) on the TwoWire object with the provided slave address. Also adds the approriate interrupt handlers
 * to the TwoWire object for processing master writes and reads.
 */
template <typename WIRE, typename ENCODER>
void EncoderInterface<WIRE,ENCODER>::begin(uint8_t slaveAddress) {
  EncoderInterface<WIRE,ENCODER>::singleton = this;
  // TODO: we call begin(..) here for the Wire library, but we don't do this for all our dependencies. This should be
  // made consistent.
  this->twoWire.begin(slaveAddress);
  this->twoWire.onReceive(&EncoderInterface<WIRE,ENCODER>::receiveEvent);
  this->twoWire.onRequest(&EncoderInterface<WIRE,ENCODER>::requestEvent);
}

/**
 * Updates the registers based on what the master wrote to this device. We don't do this in the
 * receiveEvent(..) interrupt handler as it is generally best practice to minimize the amount of time spent
 * in an interrupt handler.
 */
template <typename WIRE, typename ENCODER>
void EncoderInterface<WIRE,ENCODER>::update(const EncoderState& state) {
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
    this->setBusyStatus(0);
  }

  if( state.calibrated ) {
    this->registers[STATUS_REGISTER] &= ~UNCALIBRATED_STATUS;
  } else {
    this->registers[STATUS_REGISTER] |= UNCALIBRATED_STATUS;
  }
  
  this->registers[POSITION_REGISTER] = state.position;
  this->registers[STATE_REGISTER] = state.encoderState;
}

/**
 * Call this from the main loop. Triggers the updating of state by reading data written from master device, taking
 * appropriate actions given register values, and calls update(..) on the encoder.
 */
template <typename WIRE, typename ENCODER>
void EncoderInterface<WIRE,ENCODER>::update() {
  static uint8_t calibrating = 0;
  uint8_t newCalibratingValue = this->getCalibrationMode();

  // update registers
  this->update(this->encoder.getState());

  // Are we starting or stoping calibration (or neither)? 
  if( !calibrating && newCalibratingValue ) {
    this->encoder.startCalibration();
  } else if( calibrating && !newCalibratingValue ) {
    // TODO: What do we do if calibration is stopped mid way through?
    //this->encoder.stopCalibration();
  }
  calibrating = newCalibratingValue;

  // Reset the home position if needed.
  if( this->getResetHomeMode() ) {
    this->encoder.resetPosition();
    this->setResetHomeMode(0);
  }

  // update encoder
  this->encoder.update();
}

/**
 * Interrupt handler for when master device reads data. Will allow reading of registers starting from the register 
 * number previously written in the first byte of from the master device. If the master device has yet to write data,
 * then reading will start with reigster 0.
 */
template <typename WIRE, typename ENCODER>
void EncoderInterface<WIRE,ENCODER>::requestEvent() {
  EncoderInterface<WIRE,ENCODER>::singleton->twoWire.write(
    EncoderInterface<WIRE,ENCODER>::singleton->registers + EncoderInterface<WIRE,ENCODER>::singleton->receivedData[0], 
    NUMBER_REGISTERS - EncoderInterface<WIRE,ENCODER>::singleton->receivedData[0]);
}

/**
 * Interrupt handler for when master device writes data. The first byte of data indicates to the register number to 
 * start writing data into. The remaining bytes are written to registers in sequence. If a register cannot be written 
 * to, then the data that was attempted to be written there will be dropped. If there are insufficient registers for
 * all the data being written, then the extra bytes will be dropped. If the status is currently Busy, then all bytes
 * written will be dropped.
 */
template <typename WIRE, typename ENCODER>
void EncoderInterface<WIRE,ENCODER>::receiveEvent(int bytesReceived) {
  uint8_t bytesRead = 0;
  if( !EncoderInterface<WIRE,ENCODER>::singleton->getBusyStatus() ) {
    EncoderInterface<WIRE,ENCODER>::singleton->setBusyStatus(1);
    for( ; bytesRead < bytesReceived && bytesRead < MAX_WRITE_BYTES; bytesRead++ ) {
      EncoderInterface<WIRE,ENCODER>::singleton->receivedData[bytesRead] = 
        EncoderInterface<WIRE,ENCODER>::singleton->twoWire.read();
    }
    EncoderInterface<WIRE,ENCODER>::singleton->bytesReceivedFromMaster = bytesReceived;
  }
  for( uint8_t i = 0; i < bytesReceived - bytesRead; i++ ) {
    EncoderInterface<WIRE,ENCODER>::singleton->twoWire.read();
  }
}

} // end namespace JTIncrementalEncoder

#ifndef JTIncrementalEncoder_SKIP_DEFAULT_TEMPLATE_INSTANTIATION
#include <Wire.h>
#include <EEPROM.h>
namespace JTIncrementalEncoder {
template class EncoderInterface<TwoWire, Encoder<EEPROMClass, Print> >;
}
#endif



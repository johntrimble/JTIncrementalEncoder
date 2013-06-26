#include <vector>
#include <Wire.h>
#include "interface.h"

namespace JTIncrementalEncoder {

static void requestEvent();
static void receiveEvent(int bytesReceived);

void EncoderInterface::set_calibration_mode_sequence(std::vector<uint8_t> &vals) {
  this->calibration_mode_sequence_values = vals;
}

EncoderInterface::EncoderInterface(TwoWire& twoWire) : twoWire(twoWire) {

}

void EncoderInterface::begin(uint8_t slaveAddress) {

}

/**
 * Updates the registers based on what the master wrote to this device. We don't do this in the
 * I2C interrupt handler as it is generally best practice to minimize the amount of time spent
 * in an interrupt handler.
 */
void EncoderInterface::update(EncoderState& state) {
}

void EncoderInterface::update() {
}

uint8_t EncoderInterface::getCalibrationMode() {
  uint8_t rval = this->calibration_mode_sequence_values[0];
  this->calibration_mode_sequence_values.erase(calibration_mode_sequence_values.begin());
  return rval;
}

void EncoderInterface::setCalibrationMode(uint8_t val) {
  
}

uint8_t EncoderInterface::getResetHomeMode() {
  return 0;
}

void EncoderInterface::setResetHomeMode(uint8_t val) {

}

uint8_t EncoderInterface::getCalibratedStatus() {
  return 0;
}

void EncoderInterface::setCalibratedStatus(uint8_t val) {
  
}

void EncoderInterface::requestEvent() {
  
}

void EncoderInterface::receiveEvent(int bytesReceived) {

}

/**
 * I2C handler for when master is requesting data from this slave.
 */
static void requestEvent() {

}

/**
 * I2C handler for when master is writing data to this slave. Format is as follows:
 * - First byte is always the register address byte.
 * - Remaining bytes are to be written to registers, starting at the provided address.
 * If more data is provided than will fit in to the write registers, the excess data is sliently
 * dropped.
 */
static void receiveEvent(int bytesReceived) {

}

class EncoderInterface EncoderInterface(Wire);

}

#include "Arduino.h"
#include "jt_calibrator.h"

namespace JTIncrementalEncoder {

template <typename LOG>
static void printBin(LOG& log, uint8_t value) {
  for( int i = 7; i >= 0; i-- ) {
    log.print((int)((value >> i)&1));
  }
}

template <typename LOG>
static void printlnBin(LOG& log, uint8_t value) {
  printBin(log, value);
  log.println("");
}

const uint8_t UNSET_VALUE=16;

template <typename AnalogOutT, typename InterruptT, typename DigitalInputT, typename LOG>
ReferenceCalibrator<AnalogOutT, InterruptT, DigitalInputT, LOG>::ReferenceCalibrator(
  AnalogOutT& analogVoltage,
  InterruptT& interrupt,
  DigitalInputT& input,
  LOG& log,
  unsigned long intervalLength,
  unsigned long settlingTime)
    : _analogVoltage(analogVoltage), 
      _interrupt(interrupt), 
      _input(input), 
      log(log),
      _intervalLength(intervalLength), 
      _settlingTime(settlingTime),
      _waitReferenceStable(0),
      _paused(0) { }

template <typename AnalogOutT, typename InterruptT, typename DigitalInputT, typename LOG>
ReferenceCalibrator<AnalogOutT, InterruptT, DigitalInputT, LOG>::~ReferenceCalibrator() {
  _interrupt.setInterruptHandler(NULL, CHANGE);
}

template <typename AnalogOutT, typename InterruptT, typename DigitalInputT, typename LOG>
void ReferenceCalibrator<AnalogOutT, InterruptT, DigitalInputT, LOG>::pause(unsigned long currentTime) {
  this->_paused = 1;
  // disable the interrupt as the reference will be temporarily unstable
  this->_interrupt.setInterruptHandler(NULL, CHANGE);
}

template <typename AnalogOutT, typename InterruptT, typename DigitalInputT, typename LOG>
void ReferenceCalibrator<AnalogOutT, InterruptT, DigitalInputT, LOG>::resume(unsigned long currentTime) {
  this->_paused = 0;
  // update the value and its change time so that we don't have to wait for 
  // the next interrupt
  this->_lastChangeTime = currentTime;
  this->_currentValue = this->_input.read();
  // enable the interrupt again now that we are stable
  this->_interrupt.setInterruptHandler(this, CHANGE);
}

template <typename AnalogOutT, typename InterruptT, typename DigitalInputT, typename LOG>
void ReferenceCalibrator<AnalogOutT, InterruptT, DigitalInputT, LOG>::updateReferenceValue(uint8_t value, unsigned long currentTime) {
  ///this->pause(currentTime); // don't measure while reference stabalizing
  this->_interrupt.setInterruptHandler(NULL, CHANGE);
  this->_waitReferenceStable = 1;
  // do the actual update
  this->_referenceValue = value;
  this->_analogVoltage.write(value);
  // update the reference last update time
  this->_lastReferenceUpdateTime = currentTime;
}

template <typename AnalogOutT, typename InterruptT, typename DigitalInputT, typename LOG>
uint8_t ReferenceCalibrator<AnalogOutT, InterruptT, DigitalInputT, LOG>::checkReferenceStable(unsigned long currentTime) {
  if( this->_waitReferenceStable && (this->_settlingTime < currentTime - this->_lastReferenceUpdateTime) ) {
    // allow measurements to continue
    this->_waitReferenceStable = 0;
    this->_lastChangeTime = currentTime;
    this->_currentValue = this->_input.read();
    this->_interrupt.setInterruptHandler(this, CHANGE);
  }
  return !(this->_waitReferenceStable);
}

template <typename AnalogOutT, typename InterruptT, typename DigitalInputT, typename LOG>
void ReferenceCalibrator<AnalogOutT, InterruptT, DigitalInputT, LOG>::begin(unsigned long currentTime) {
  this->_lastChangeTime = currentTime;
  this->_startTime = currentTime;
  this->_currentValue = UNSET_VALUE; // bogus value (indicates unset
  this->_highTime = 0;
  this->_lowTime = 0;
  this->_finished = 0;
  this->_mask = 0x80;
  this->updateReferenceValue(0x80, currentTime);
}

template <typename AnalogOutT, typename InterruptT, typename DigitalInputT, typename LOG>
void ReferenceCalibrator<AnalogOutT, InterruptT, DigitalInputT, LOG>::interruptOccurred() {
  this->_currentValue = this->_input.read();
}

template <typename AnalogOutT, typename InterruptT, typename DigitalInputT, typename LOG>
uint8_t ReferenceCalibrator<AnalogOutT, InterruptT, DigitalInputT, LOG>::isReferenceStable(unsigned long currentTime) {
  if( this->_settlingTime < currentTime - this->_lastReferenceUpdateTime ) {
    return 1;
  } else {
    return 0;
  }
}

template <typename AnalogOutT, typename InterruptT, typename DigitalInputT, typename LOG>
uint8_t ReferenceCalibrator<AnalogOutT, InterruptT, DigitalInputT, LOG>::isFinished() {
  return this->_finished;
}

template <typename AnalogOutT, typename InterruptT, typename DigitalInputT, typename LOG>
uint8_t ReferenceCalibrator<AnalogOutT, InterruptT, DigitalInputT, LOG>::isPaused() {
  return this->_paused;
}

template <typename AnalogOutT, typename InterruptT, typename DigitalInputT, typename LOG>
uint8_t ReferenceCalibrator<AnalogOutT, InterruptT, DigitalInputT, LOG>::isIntervalDone(unsigned long currentTime) {
  if( this->_lowTime + this->_highTime > this->_intervalLength ) {
    return 1; 
  } else {
    return 0;
  }
}

template <typename AnalogOutT, typename InterruptT, typename DigitalInputT, typename LOG>
void ReferenceCalibrator<AnalogOutT, InterruptT, DigitalInputT, LOG>::updateHighLowTime(unsigned long currentTime) {
  if( this->_currentValue != UNSET_VALUE ) {
    unsigned long delta = currentTime - this->_lastChangeTime;
    if( this->_currentValue == HIGH ) {
      this->_highTime += delta;
    } else {
      this->_lowTime += delta;
    }
    this->_lastChangeTime = currentTime;
  }
}

template <typename AnalogOutT, typename InterruptT, typename DigitalInputT, typename LOG>
uint8_t ReferenceCalibrator<AnalogOutT, InterruptT, DigitalInputT, LOG>::update(unsigned long currentTime) {
  if( !(this->_finished) ) {
    this->checkReferenceStable(currentTime);
    if( !(this->_paused || this->_waitReferenceStable) ) {

      this->updateHighLowTime(currentTime);
      
      // check if we are done with this interval
      if( this->isIntervalDone(currentTime) ) {
        uint8_t newReferenceValue = this->_referenceValue;
        uint8_t dutyCycleAbove50;
        log.print("High time: "); log.println(_highTime);
        log.print("Low time: "); log.println(_lowTime);
        if( this->_highTime > this->_lowTime ) {
          // we have measurements, and they are mostly high
          dutyCycleAbove50 = 1;
        } else {
          // we have measurements, and they are mostly low
          dutyCycleAbove50 = 0; 
        }
        
        if( dutyCycleAbove50 ) {
          // unset the bit
          log.print("Unsetting bit: ");printBin(log, this->_referenceValue); log.print(" -> ");
          newReferenceValue = this->_referenceValue ^ this->_mask;
          printlnBin(log, newReferenceValue);
        } else {
          // all good, leave the bit set 
        }
        
        if( this->_mask == 1 ) {
          // we are done
          this->_finished = 1;
          log.println("Calibration finished");
          this->_interrupt.setInterruptHandler(NULL, CHANGE);
        } else {
          // update the mask and reference for next iteration
          this->_mask = (this->_mask >> 1);
          //log.print("Current mask: ");printlnBin(this->mask);
          log.print("Setting next bit: ");printBin(log, this->_referenceValue); log.print(" -> ");
          newReferenceValue = newReferenceValue | this->_mask;
          printlnBin(log, newReferenceValue);
          
          // reset high and low time
          this->_highTime = 0;
          this->_lowTime = 0;
        }
        log.print("Reference set to: "); printlnBin(log, newReferenceValue);
        // update the analog voltage if needed
        if( this->_referenceValue != newReferenceValue ) {
          this->updateReferenceValue(newReferenceValue, currentTime);
        }
      }
    }
  }
  return this->_finished;
}

} // end JTIncrementalEncoder

#ifndef JTIncrementalEncoder_SKIP_DEFAULT_TEMPLATE_INSTANTIATION
#include "jt_pin.h"
namespace JTIncrementalEncoder {
template class ReferenceCalibrator<AnalogOutputPin, Interrupt, DigitalInputPin, Print>;
}
#endif


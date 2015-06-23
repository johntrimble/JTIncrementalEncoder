#ifndef jt_calibrator_h
#define jt_calibrator_h

#include "jt_interrupt.h"

namespace JTIncrementalEncoder {
  
template <typename AnalogOutT, typename InterruptT, typename DigitalInputT, typename LOG>
class ReferenceCalibrator : public JTIncrementalEncoder::InterruptHandler {
private:
  AnalogOutT& _analogVoltage;
  InterruptT& _interrupt;
  DigitalInputT& _input;
  LOG& log;
  unsigned long _highTime;
  unsigned long _lowTime;
  unsigned long _lastChangeTime;
  unsigned long _lastReferenceUpdateTime;
  unsigned long _startTime;
  unsigned long _intervalLength;
  unsigned long _settlingTime;
  volatile uint8_t _currentValue;
  uint8_t _referenceValue;
  uint8_t _mask;
  uint8_t _waitReferenceStable;
  uint8_t _finished;
  uint8_t _paused;

  uint8_t isIntervalDone(unsigned long currentTimeInMillis);
  void updateHighLowTime(unsigned long currentTimeInMillis);  
  uint8_t isReferenceStable(unsigned long currentTimeInMillis);
  void interruptOccurred();
  uint8_t checkReferenceStable(unsigned long currentTimeInMillis);
  void updateReferenceValue(uint8_t value, unsigned long currentTimeInMillis);

public:
  ReferenceCalibrator(
    AnalogOutT& analogVoltage,
    InterruptT& interrupt,
    DigitalInputT& input,
    LOG& log,
    unsigned long intervalLength,
    unsigned long settlingTime);

  ~ReferenceCalibrator();
  
  void pause(unsigned long currentTimeInMillis);
  
  void resume(unsigned long currentTimeInMillis);
  
  void begin(unsigned long currentTimeInMillis);
  
  uint8_t isFinished();

  uint8_t isPaused();
  
  uint8_t update(unsigned long currentTimeInMillis);
};
} // end JTIncrementalEncoder

#ifndef JTIncrementalEncoder_SKIP_DEFAULT_TEMPLATE_INSTANTIATION
#include "Arduino.h"
#include "jt_pin.h"
namespace JTIncrementalEncoder {
typedef ReferenceCalibrator<AnalogOutputPin, Interrupt, DigitalInputPin, Print> DefaultReferenceCalibrator;
}
#endif

#endif

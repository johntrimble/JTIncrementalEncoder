// Prevent default template instantiation 
#define JTIncrementalEncoder_SKIP_DEFAULT_TEMPLATE_INSTANTIATION 1

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include <bitset>
#include <string>
#include <cmath>
#include <cstdlib>

#include "jt_calibrator.cpp"

using namespace JTIncrementalEncoder;

// our own implementation of micros(...) and millis(...) for testing
static unsigned long currentMicros = 0;
unsigned long micros() {
  return currentMicros;
}

static unsigned long currentMillis = 0;
unsigned long millis() {
  return currentMillis;
}

class NopLog {
public:
  size_t println(const char s[]) { return 0; }
  size_t println(unsigned char c) { return 0; }
  size_t print(const char s[]) { return 0; }
  size_t print(unsigned char) { return 0; }
};

class MockAnalogOut {
public:
  MOCK_METHOD1(write, void(uint8_t));
};

class StubAnalogOut {
public:
  uint8_t value;

  StubAnalogOut()
    : value(0) { }

  void write(uint8_t value) {
    this->value = value;
  }
};

class MockInterrupt {
public:
  MOCK_METHOD0(disable, void(void));
  MOCK_METHOD0(enable, void(void));
  MOCK_METHOD2(setInterruptHandler, void(InterruptHandler*, int));
};

class StubInterrupt {
public:
  uint8_t enabled;
  InterruptHandler* handler;

  StubInterrupt()
    : enabled(0),
      handler(NULL) {
  }

  void disable() {
    enabled = 0;
  }

  void enable() {
    enabled = 1;
  }

  void setInterruptHandler(InterruptHandler* handler, int mode) {
    this->handler = handler;
    if( handler == NULL ) {
      this->disable();
    } else {
      this->enable();
    }
  }
};

class MockDigitalInput {
public:
  MOCK_METHOD0(read, uint8_t(void));
};

class StubDigitalInput {
public:
  uint8_t value;

  StubDigitalInput()
    : value(0) { }

  uint8_t read() {
    return value;
  }
};

typedef ReferenceCalibrator<
    StubAnalogOut, 
    StubInterrupt, 
    StubDigitalInput, 
    NopLog> 
  ReferenceCalibratorWithMocks;

class CalibratorTestSuite : public ::testing::Test {
public:
  StubAnalogOut* analogOut;
  StubInterrupt* interrupt;
  StubDigitalInput* digitalIn;
  NopLog* log;
  ReferenceCalibratorWithMocks* calibrator;
  
  void SetUp() {
    analogOut = new StubAnalogOut();
    interrupt = new StubInterrupt();
    digitalIn = new StubDigitalInput();
    log = new NopLog();
    calibrator = new ReferenceCalibratorWithMocks(
      *analogOut, 
      *interrupt, 
      *digitalIn, 
      *log, 
      1000, 
      500);
  }

  void TearDown() {
    delete calibrator;
    delete log;
    delete digitalIn;
    delete interrupt;
    delete analogOut;
  }
};

TEST_F(CalibratorTestSuite, begin) {
  calibrator->begin(0);
  // we just got started, so we shouldn't be finished
  ASSERT_FALSE(calibrator->isFinished());
  // initially, we should have interrupts disabled as we will be waiting for
  // the reference voltage to stabalize
  ASSERT_THAT(interrupt->handler, testing::Ne(calibrator));
  ASSERT_THAT(analogOut->value, testing::Eq(0x80));
}

void simulateSignal(
  ReferenceCalibratorWithMocks& calibrator,
  StubInterrupt& interrupt,
  StubAnalogOut& analogOut,
  StubDigitalInput& digitalIn,
  unsigned long sineFrequency,
  uint8_t amplitude,
  uint8_t centerOfAmplitude,
  unsigned long updateCallFrequency,
  unsigned long durationMicros,
  unsigned long currentTimeMicros) {

  unsigned long endTimeMicros = currentTimeMicros+durationMicros;
  unsigned long sinePeriodMicros = (1.0 / (double)sineFrequency)*1000*1000;
  unsigned long updatePeriodMicros = (1.0 / (double)updateCallFrequency)*1000*1000;
  while( currentTimeMicros < endTimeMicros ) {
    unsigned long nextUpdateTimeMicros = currentTimeMicros+updatePeriodMicros;
    for( ; currentTimeMicros < nextUpdateTimeMicros; currentTimeMicros += 100 ) {
      double rawValue = centerOfAmplitude + amplitude*sin(currentTimeMicros * ((2*M_PI)/sinePeriodMicros));
      uint8_t value = rawValue > analogOut.value ? LOW : HIGH;
      if( value != digitalIn.value ) {
        digitalIn.value = value;
        if( interrupt.handler ) {
          interrupt.handler->interruptOccurred();
        }
      }
    }
    calibrator.update(currentTimeMicros / 1000);
  }
}

TEST_F(CalibratorTestSuite, update) {
  // acceptable amount of error in center of amplitude estimate where error
  // equals abs( (centerOfAmplitude - estimate) / ampliutde )
  double acceptableError = 0.25;

  unsigned long sineFrequencies[] = {100, 200};
  int length = sizeof(sineFrequencies) / sizeof(unsigned long);
  for( int i = 0; i < length; i++ ) {
    for(uint8_t centerOfAmplitude = 10; centerOfAmplitude < 240; centerOfAmplitude += 10 ) {
      for( 
        uint8_t amplitude = 9;
        amplitude < min(255 - centerOfAmplitude, centerOfAmplitude);
        amplitude += 10 ) {
          //uint8_t centerOfAmplitude = 40;
          unsigned long sineFrequency = sineFrequencies[i];
          calibrator->begin(0);
          simulateSignal(
            *calibrator,
            *interrupt,
            *analogOut,
            *digitalIn,
            sineFrequency, // sine wave signal frequency
            amplitude, // amplitude
            centerOfAmplitude, // center of amplitude
            500, // update call frequency
            20*1000*1000, // duration in microseconds (20 seconds)
            0 // current time in microseconds
            );
          // reference voltage should be *roughly* equal to center of amplitude
          uint8_t ref = analogOut->value;
          double error = ((double)ref - (double)centerOfAmplitude) / (double)amplitude;
          ASSERT_THAT(abs(error), testing::Le(acceptableError));
      }
    }
  }
}

TEST_F(CalibratorTestSuite, pause) {
  unsigned long durationMicros = 20*1000*1000;
  uint8_t centerOfAmplitude = 128;
  uint8_t amplitude = 127;
  unsigned long sineFrequency = 100;
  unsigned currentTimeMicros = 0;
  calibrator->begin(currentTimeMicros);
  calibrator->pause(currentTimeMicros / 1000);
  ASSERT_TRUE(calibrator->isPaused());
  ASSERT_FALSE(calibrator->isFinished());
  simulateSignal(
    *calibrator,
    *interrupt,
    *analogOut,
    *digitalIn,
    sineFrequency, // sine wave signal frequency
    amplitude, // amplitude
    centerOfAmplitude, // center of amplitude
    500, // update call frequency
    durationMicros, // duration in microseconds (20 seconds)
    currentTimeMicros // current time in microseconds
    );
  ASSERT_TRUE(calibrator->isPaused());
  // when paused, we should ignore any input signal and so should still not
  // be finished calibrating
  ASSERT_FALSE(calibrator->isFinished());
}

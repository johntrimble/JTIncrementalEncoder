// Prevent default template instantiation 
#define JTIncrementalEncoder_SKIP_DEFAULT_TEMPLATE_INSTANTIATION 1

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include <bitset>
#include <string>
#include "encoder.cpp"
#include "Arduino.h"
#include "MCP42xxx.h"

using namespace JTIncrementalEncoder;

// Channel A
static const int CHANNEL_A_INPUT_PIN = 2;
static const int CHANNEL_A_INTERRUPT = 0;
static const int CHANNEL_A_RAW_INPUT_PIN = 5;

// Channel B
static const int CHANNEL_B_INPUT_PIN = 3;
static const int CHANNEL_B_INTERRUPT = 1;
static const int CHANNEL_B_RAW_INPUT_PIN = 6;

void MCP42xxx::write(MCP42xxx::Channel channel, uint8_t value) { }

class MockPot {
public:
  MOCK_METHOD2(write, void(MCP42xxx::Channel channel, uint8_t value));
};

class NopLog {
public:
  size_t println(const char s[]) { return 0; }
  size_t println(unsigned char c) { return 0; }
  size_t print(const char s[]) { return 0; }
  size_t print(unsigned char) { return 0; }
};

class MockStorage {
public:
  MOCK_METHOD1(read, uint8_t(int addr));
  MOCK_METHOD2(write, void(int addr, uint8_t val));
};

class EncoderTestSuite : public ::testing::Test {
public:
  Encoder<MockPot, MockStorage, NopLog> *encoder;
  NopLog *log;
  MockStorage *storage;
  MockPot *pot;

  void SetUp() {
    pot = new MockPot();
    storage = new MockStorage();
    log = new NopLog();
    encoder = new Encoder<MockPot, MockStorage, NopLog>(
      *pot, 
      *storage, 
      *log,  
      CHANNEL_A_INPUT_PIN, 
      CHANNEL_A_RAW_INPUT_PIN,
      MCP42xxx::CHANNEL_0,
      CHANNEL_A_INTERRUPT,
      NULL,
      CHANNEL_B_INPUT_PIN,
      CHANNEL_B_RAW_INPUT_PIN,
      MCP42xxx::CHANNEL_1,
      CHANNEL_B_INTERRUPT,
      NULL);
  }

  void TearDown() {
    delete encoder;
    delete log;
    delete storage;
    delete pot;
  }
};

TEST_F(EncoderTestSuite, getDirection_CLOCKWISE) {
  EXPECT_EQ(CLOCKWISE, this->encoder->getDirection(0x12)); // 00010010
  EXPECT_EQ(CLOCKWISE, this->encoder->getDirection(0x79)); // 01111001
  EXPECT_EQ(CLOCKWISE, this->encoder->getDirection(0x8D)); // 10001101
  EXPECT_EQ(CLOCKWISE, this->encoder->getDirection(0xE6)); // 11100110
}

TEST_F(EncoderTestSuite, getDirection_COUNTERCLOCKWISE) {
  EXPECT_EQ(COUNTERCLOCKWISE, this->encoder->getDirection(0xD2)); // 11010010
  EXPECT_EQ(COUNTERCLOCKWISE, this->encoder->getDirection(0x49)); // 01001001
  EXPECT_EQ(COUNTERCLOCKWISE, this->encoder->getDirection(0xBD)); // 10111101
  EXPECT_EQ(COUNTERCLOCKWISE, this->encoder->getDirection(0xD6)); // 11010110
}

TEST_F(EncoderTestSuite, updateChannelAEncoderState) {
  // set up the channel
  EncoderChannel channel;
  channel.inputPin = 8;

  // set up the value that will be read from the channel's input pin
  std::vector<uint8_t> values;
  values.push_back(1);
  values.push_back(0);
  values.push_back(0);
  pinMock.set_digital_value_sequence(8, values);

  // check read high value
  std::bitset<8> encoder_state_bitset(std::string("00100011"));
  volatile uint8_t encoder_state = encoder_state_bitset.to_ulong();
  uint8_t index = 4;

  updateChannelAEncoderState(channel, encoder_state, index);

  EXPECT_EQ(std::string("10110100"), std::bitset<8>(encoder_state).to_string());
  EXPECT_EQ(5, index);

  // check read low value
  updateChannelAEncoderState(channel, encoder_state, index);

  EXPECT_EQ(std::string("11100101"), std::bitset<8>(encoder_state).to_string());
  EXPECT_EQ(6, index);

  // check that the index doesn't overflow into the encoder values
  index = 16;
  encoder_state_bitset = std::bitset<8>(std::string("00101111"));
  encoder_state = encoder_state_bitset.to_ulong();

  updateChannelAEncoderState(channel, encoder_state, index);

  EXPECT_EQ(std::string("10100000"), std::bitset<8>(encoder_state).to_string());
  EXPECT_EQ(17, index);
}

TEST_F(EncoderTestSuite, updateChannelBEncoderState) {
  // set up the channel
  EncoderChannel channel;
  channel.inputPin = 8;

  // set up the value that will be read from the channel's input pin
  std::vector<uint8_t> values;
  values.push_back(1);
  values.push_back(0);
  values.push_back(0);
  pinMock.set_digital_value_sequence(8, values);

  // check read high value
  std::bitset<8> encoder_state_bitset(std::string("00010011"));
  volatile uint8_t encoder_state = encoder_state_bitset.to_ulong();
  uint8_t index = 4;

  updateChannelBEncoderState(channel, encoder_state, index);

  EXPECT_EQ(std::string("01110100"), std::bitset<8>(encoder_state).to_string());
  EXPECT_EQ(5, index);

  // check read low value
  updateChannelBEncoderState(channel, encoder_state, index);

  EXPECT_EQ(std::string("11010101"), std::bitset<8>(encoder_state).to_string());
  EXPECT_EQ(6, index);

  // check that the index doesn't overflow into the encoder values
  index = 16;
  encoder_state_bitset = std::bitset<8>(std::string("00101111"));
  encoder_state = encoder_state_bitset.to_ulong();

  updateChannelBEncoderState(channel, encoder_state, index);

  EXPECT_EQ(std::string("10000000"), std::bitset<8>(encoder_state).to_string());
  EXPECT_EQ(17, index);
}

typedef struct {
  unsigned long measureTime;
  float rawValue;
  uint8_t value;
} Sample;

void generateWaveSamples(
  unsigned long step, 
  unsigned int count,
  float threshold,
  float frequency, 
  float amplitude, 
  float phase, 
  float center,
  std::vector<Sample>& samples) {

  for(unsigned int i = 0; i < count; i++ ) {
    Sample sample;
    sample.measureTime = (i+1)*step;
    sample.rawValue = amplitude
      * sin( 2 * M_PI * frequency * ( sample.measureTime / 1000000.0 ) + phase )
      + center;
    sample.value = (sample.rawValue > threshold)? HIGH : LOW;
    samples.push_back(sample);
  }
}

void checkDutyCycleMovingAverage(
  unsigned long step,
  int f,
  int t,
  double threshold,
  unsigned int count) {

  DutyCycleMovingAverage dutyCycleMA;
  unsigned long timeUntilAverageReadyMicros =  DUTY_CYCLE_MA_SAMPLE_LENGTH 
    * MOVING_AVERAGE_DATA_SIZE + DUTY_CYCLE_MA_SAMPLE_LENGTH - 1;

  std::vector<Sample> samples;
 
  generateWaveSamples(step, count, threshold, f / 2.0, t, 0, 0, samples);

  initDutyCycleMovingAverage(dutyCycleMA, 0);

  for( unsigned int i = 0; i < count; i++ ) {
    updateDutyCycleMA(dutyCycleMA, samples[i].value, samples[i].measureTime);
  }

  unsigned long startTime = samples[count - 1].measureTime;
  if( startTime > timeUntilAverageReadyMicros ) {
    startTime -= timeUntilAverageReadyMicros;
  } else {
    startTime = 0;
  }

  unsigned long highCount = 0;
  unsigned long totalCount = 0;
  float expectedDutyCycle;
  for(unsigned int i = 0; i < count; i++ ) {
    if( samples[i].measureTime >= startTime ) {
      if( samples[i].value == HIGH )
        highCount++;
      totalCount++;
    }
  }
  expectedDutyCycle = highCount/(float)totalCount;

  EXPECT_TRUE(dutyCycleMA.movingAverage.ready);
  EXPECT_NEAR(
    expectedDutyCycle, 
    dutyCycleMA.movingAverage.average/(float)255, 
    0.10);

  if( expectedDutyCycle > 0.5 ) {
    EXPECT_TRUE(isDutyCycleGreatherThan50Percent(dutyCycleMA));
  } else {
    EXPECT_FALSE(isDutyCycleGreatherThan50Percent(dutyCycleMA));
  }
}

TEST_F(EncoderTestSuite, dutyCycleMovingAverage) {
  unsigned long timeUntilAverageReadyMicros = DUTY_CYCLE_MA_SAMPLE_LENGTH 
    * MOVING_AVERAGE_DATA_SIZE + DUTY_CYCLE_MA_SAMPLE_LENGTH - 1;
  const int numberThresholds = 4;
  const int numberCounts = 2;
  const int numberIntervals = 2;
  float thresholds[numberThresholds] = {0.75,-0.75, 0.20, -0.20};
  float counts[numberCounts] = {1000, 1500};
  unsigned long intervals[numberIntervals] = {
    (timeUntilAverageReadyMicros / 1000.0), 
    (timeUntilAverageReadyMicros / 432.0)};
  for( int i = 0; i < numberThresholds; i++ ) {
    for( int j = 0; j < numberCounts; j++ ) {
      for( int k = 0; k < numberIntervals; k++ ) {
        checkDutyCycleMovingAverage(intervals[k], 
          5, 1, thresholds[i], counts[j]);
      }
    }
  }
}

void timeShiftSamples(std::vector<Sample>& samples, unsigned long timeMicros) {
  for( std::vector<Sample>::iterator it = samples.begin(); 
    it != samples.end(); 
    ++it ) {

    (*it).measureTime += timeMicros;
  }
}

TEST_F(EncoderTestSuite, updateCalibration) {
  ChannelCalibration calibration;
  unsigned long timeUntilIterationDoneMicros = 
    CALIBRATION_INTERVAL_LENGTH;
  unsigned long interval = (timeUntilIterationDoneMicros / 1000.0) + 1;
  std::vector<Sample> samples;
  unsigned int sampleIdx = 0;

  generateWaveSamples(
    interval, 
    1000,
    2.5, // threshold
    2.5, // frequency
    1, // amplitude
    0, // phase
    4, // center of amplitude
    samples);

  initChannelCalibration(calibration, 0);

  // check phase 1 of calibration
  for( ; sampleIdx < 1000; sampleIdx++ ) {
    // mask and pot should stay at 128 until phase 1 is done, which happens in
    // CALIBRATION_INTERVAL_LENGTH microseconds
    EXPECT_EQ(
      std::string("10000000"), 
      std::bitset<8>(calibration.mask).to_string());
    EXPECT_EQ(
        std::string("10000000"), 
        std::bitset<8>(calibration.potSetting).to_string());
    updateChannelCalibration(
            calibration, 
            samples[sampleIdx].value, 
            samples[sampleIdx].measureTime);
  }

  // clean up
  samples.clear();

  // check that we moved the mask over one bit
  EXPECT_EQ(
    std::string("01000000"), 
    std::bitset<8>(calibration.mask).to_string());

  // check that the resistance setting for the POT has the most significant bit
  // set, since we need to decrease resistance (and raise reference voltage), 
  // and that the next bit over is also set since we are in the second phase of 
  // calibration.
  EXPECT_EQ(
    std::string("11000000"),
    std::bitset<8>(calibration.potSetting).to_string());

  // calibration not done
  EXPECT_FALSE(calibration.finished);

  // We will simulate changes in the reference voltage by adjusting the
  // threshold for the generated signal.
  generateWaveSamples(
    interval, 
    1000,
    (calibration.potSetting / 255.0) * 5, // threshold
    2.5, // frequency
    1, // amplitude
    0, // phase
    4, // center of amplitude
    samples);

  timeShiftSamples(samples, interval*1000);
  
  // check phase 2 of calibration
  for( int i = 0; i < 1000; i++ ) {
    // mask and pot should stay at 64 until phase 2 is done, which happens in
    // CALIBRATION_INTERVAL_LENGTH microseconds
    EXPECT_EQ(
      std::string("01000000"), 
      std::bitset<8>(calibration.mask).to_string());
    EXPECT_EQ(
        std::string("11000000"),
        std::bitset<8>(calibration.potSetting).to_string());
    updateChannelCalibration(
            calibration, 
            samples[i].value, 
            samples[i].measureTime);
  }

  // clean up
  samples.clear();

  // check that we moved to the next bit
  EXPECT_EQ(
    std::string("00100000"),
    std::bitset<8>(calibration.mask).to_string());

  // check that we left the previous bit set and set the next one for the next
  // phase
  EXPECT_EQ(
    std::string("11100000"),
    std::bitset<8>(calibration.potSetting).to_string());

  // calibration not done
  EXPECT_FALSE(calibration.finished);

  // do the remaining 6 phases
  for(int phase = 2; phase < 8; phase++ ) {
    float threshold = (calibration.potSetting / 255.0)*5;
    generateWaveSamples(
      interval, 
      1000,
      threshold,
      2.5, // frequency
      1, // amplitude
      0, // phase
      4, // center of amplitude
      samples);
    timeShiftSamples(samples, interval*1000*phase);

    for( int i = 0; i < 1000; i++ ) {
      updateChannelCalibration(
              calibration, 
              samples[i].value, 
              samples[i].measureTime);
    }
    // std::cout << "POT: " << std::bitset<8>(calibration.potSetting) << "\n";
    samples.clear();
  }

  // after the 8 phases run, the calibration should be finished
  EXPECT_TRUE(calibration.finished);
  // the pot setting roughly corresponds to how high the reference voltage is,
  // which should roughly converge towards the center of amplitude
  EXPECT_NEAR(4, (calibration.potSetting/255.0)*5, 0.2);
}

TEST_F(EncoderTestSuite, updatePosition) {
  int position = 5;
  // check moving forward
  updatePosition(std::bitset<8>(std::string("10110011")).to_ulong(), std::bitset<8>(std::string("11100101")).to_ulong(), position);
  EXPECT_EQ(7, position);

  // check moving backwards
  updatePosition(std::bitset<8>(std::string("11100011")).to_ulong(), std::bitset<8>(std::string("10110101")).to_ulong(), position);
  EXPECT_EQ(5, position);

  // check roll over forwards
  updatePosition(std::bitset<8>(std::string("10110011")).to_ulong(), std::bitset<8>(std::string("11100001")).to_ulong(), position);
  EXPECT_EQ(19, position);

  // check roll over backwards
  updatePosition(std::bitset<8>(std::string("11100011")).to_ulong(), std::bitset<8>(std::string("10110001")).to_ulong(), position);
  EXPECT_EQ(5, position);
}

TEST_F(EncoderTestSuite, saveSettings) {
  EXPECT_CALL(*(this->storage), write(::testing::_, ::testing::_))
    .Times(12);
  EXPECT_CALL(*(this->storage), write(5, 179))
    .Times(1);    

  // save the encoder pair
  int addr = 5;
  this->encoder->saveSettings(*(this->storage), addr);
}

TEST_F(EncoderTestSuite, loadSettings) {
  ::testing::InSequence S;
  // average, min, max
  int addr = 0;
  uint8_t data[13] = { 
    179, 
    0, 3, // 768
    72, 0, // 72
    255, 3, // 1023
    65, 1, // 321
    2, 0, // 2
    0, 3 // 768
  };
  for(int i = 0; i < 13; i++ ) {
    EXPECT_CALL(*(this->storage), read(i))
      .WillOnce(::testing::Return(data[i]));
  }
  uint8_t rvalue = this->encoder->loadSettings(*(this->storage), addr);
  EXPECT_EQ(true, rvalue);
  EXPECT_EQ(768, this->encoder->getState().channels.a.average);
  EXPECT_EQ(72, this->encoder->getState().channels.a.minValue);
  EXPECT_EQ(1023, this->encoder->getState().channels.a.maxValue);
  EXPECT_EQ(321, this->encoder->getState().channels.b.average);
  EXPECT_EQ(2, this->encoder->getState().channels.b.minValue);
  EXPECT_EQ(768, this->encoder->getState().channels.b.maxValue);
}

TEST_F(EncoderTestSuite, loadSettings_badKey) {
  int addr = 0;
  EXPECT_CALL(*(this->storage), read(addr))
    .WillOnce(::testing::Return(178));
  uint8_t rvalue = this->encoder->loadSettings(*(this->storage), addr);
  EXPECT_EQ(false, rvalue);
}

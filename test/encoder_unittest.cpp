// Prevent default template instantiation 
#define JTIncrementalEncoder_SKIP_DEFAULT_TEMPLATE_INSTANTIATION 1

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include <bitset>
#include <string>

#include "encoder.cpp"

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

class MockStorage {
public:
  MOCK_METHOD1(read, uint8_t(int addr));
  MOCK_METHOD2(write, void(int addr, uint8_t val));
};

class MockCalibrator {
public:
  MOCK_METHOD1(pause, void(unsigned long));
  MOCK_METHOD1(resume, void(unsigned long));
  MOCK_METHOD1(begin, void(unsigned long));
  MOCK_METHOD0(isFinished, uint8_t(void));
  MOCK_METHOD1(update, uint8_t(unsigned long));
};

class EncoderTestSuite : public ::testing::Test {
public:
  static const int CHANNEL_A_INPUT_PIN = 2;
  static const int CHANNEL_A_INTERRUPT = 0;
  static const int CHANNEL_B_INPUT_PIN = 3;
  static const int CHANNEL_B_INTERRUPT = 1;

  Encoder<MockCalibrator, MockStorage, NopLog> *encoder;
  NopLog *log;
  MockStorage *storage;
  MockCalibrator* calibratorA;
  MockCalibrator* calibratorB;
  Calibration<MockCalibrator> *calibration;

  void SetUp() {
    storage = new MockStorage();
    log = new NopLog();
    calibratorA = new MockCalibrator();
    calibratorB = new MockCalibrator();
    calibration = new Calibration<MockCalibrator>(*calibratorA, *calibratorB);
    encoder = new Encoder<MockCalibrator, MockStorage, NopLog>(
      *storage, 
      *log,
      *calibration,
      CHANNEL_A_INPUT_PIN, 
      CHANNEL_A_INTERRUPT,
      NULL,
      CHANNEL_B_INPUT_PIN,
      CHANNEL_B_INTERRUPT,
      NULL);
  }

  void TearDown() {
    delete encoder;
    delete calibration;
    delete calibratorB;
    delete calibratorA;
    delete log;
    delete storage;
  }
};

TEST_F(EncoderTestSuite, calibrating) {
  const ::testing::Sequence s1, s2;

  // check starting calibration
  EXPECT_CALL(*(this->calibratorA), begin(7))
    .Times(1)
    .InSequence(s1);
  EXPECT_CALL(*(this->calibratorB), begin(7))
    .Times(1)
    .InSequence(s2);

  currentMillis = 7;
  encoder->startCalibration();

  // this is an annoying bit of public state that should be removed
  ASSERT_FALSE(encoder->state.calibrated);


  // check Encoder.update() delegates to calibrator when calibrating
  EXPECT_CALL(*(this->calibratorA), update(10))
    .Times(1)
    .InSequence(s1)
    .WillRepeatedly(testing::Return(0));
  EXPECT_CALL(*(this->calibratorA), isFinished())
    .Times(1)
    .InSequence(s1)
    .WillRepeatedly(testing::Return(0)); 
  EXPECT_CALL(*(this->calibratorB), update(10))
    .Times(1)
    .InSequence(s2)
    .WillRepeatedly(testing::Return(0));
  EXPECT_CALL(*(this->calibratorB), isFinished())
    .Times(testing::AtMost(1))
    .InSequence(s2)
    .WillRepeatedly(testing::Return(0)); 
  
  currentMillis = 10;
  encoder->update();


  // check completion of calibration
  EXPECT_CALL(*(this->calibratorA), update(13))
    .Times(1)
    .InSequence(s1)
    .WillRepeatedly(testing::Return(0));
  EXPECT_CALL(*(this->calibratorA), isFinished())
    .Times(1)  
    .InSequence(s1)
    .WillRepeatedly(testing::Return(1));
  EXPECT_CALL(*(this->calibratorB), update(13))
    .Times(1)
    .InSequence(s2)
    .WillRepeatedly(testing::Return(0));
  EXPECT_CALL(*(this->calibratorB), isFinished())
    .Times(1)
    .InSequence(s2)
    .WillRepeatedly(testing::Return(1));

  currentMillis = 13;
  encoder->update();
  ASSERT_TRUE(encoder->state.calibrated);
}

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

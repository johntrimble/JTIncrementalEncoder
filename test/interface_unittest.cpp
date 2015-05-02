// Prevent default template instantiation 
#define JTIncrementalEncoder_SKIP_DEFAULT_TEMPLATE_INSTANTIATION 1

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include <bitset>
#include <string>
#include "Arduino.h"
#include "i2c_interface.cpp"

using namespace JTIncrementalEncoder;

class MockEncoder {
public:
  EncoderState state;
  const EncoderState& getState() const { return this->state; }
  MOCK_METHOD1(getDirection, void(uint8_t value));
  MOCK_METHOD0(resetPosition, void(void));
  MOCK_METHOD0(startCalibration, void(void));
  MOCK_METHOD0(stopCalibration, void(void));
  MOCK_METHOD0(update, void(void));
};

class MockTwoWire {
public:
  MOCK_METHOD1(begin, void(uint8_t slaveAddress));
  MOCK_METHOD1(onReceive, void(void (*f)(int bytesReceived)));
  MOCK_METHOD1(onRequest, void(void (*f)(void)));
  MOCK_METHOD1(write, void(uint8_t val));
  MOCK_METHOD2(write, size_t(const uint8_t *buffer, size_t size));
  MOCK_METHOD0(read, uint8_t(void));
};

class InterfaceTestSuite : public ::testing::Test {
public:
  MockEncoder *encoder;
  MockTwoWire *twoWire;
  EncoderInterface<MockTwoWire, MockEncoder> *interface;

  void SetUp() {
    this->encoder = new MockEncoder();
    this->twoWire = new MockTwoWire();
    this->interface = new EncoderInterface<MockTwoWire, MockEncoder>(*(this->twoWire), *(this->encoder));
  }

  void TearDown() {
    delete this->interface;
    delete this->twoWire;
    delete this->encoder;
  }
};

TEST_F(InterfaceTestSuite, begin) {
  EXPECT_CALL(*(this->twoWire), begin(5));
  EXPECT_CALL(*(this->twoWire), onReceive(::testing::_));
  EXPECT_CALL(*(this->twoWire), onRequest(::testing::_));
  EXPECT_FALSE((EncoderInterface<MockTwoWire,MockEncoder>::getSingleton()));
  this->interface->begin(5);
  EXPECT_TRUE((EncoderInterface<MockTwoWire,MockEncoder>::getSingleton()));
}

TEST_F(InterfaceTestSuite, calibrationMode) {
  // EXPECT_CALL(*(this->encoder), startCalibration());
  // EXPECT_CALL(*(this->encoder), update())
  //   .Times(3);
  // EXPECT_CALL(*(this->encoder), stopCalibration());
  // this->interface->setCalibrationMode(1);
  // this->interface->update();
  // this->interface->update();
  // this->interface->setCalibrationMode(0);
  // this->interface->update();
}

TEST_F(InterfaceTestSuite, ignore_master_write_when_busy) {
  ::testing::InSequence seq;
  uint8_t data[2] = {0x04, 0x04}; // offset to mode index and set calibration mode
  this->interface->setBusyStatus(1);
  EXPECT_CALL(*(this->encoder), update())
    .RetiresOnSaturation();
  this->interface->update();
  for(int i = 0; i < 2; i++ ) {
    EXPECT_CALL(*(this->twoWire), read())
      .WillOnce(::testing::Return(data[i]))
      .RetiresOnSaturation();
  }
  this->interface->receiveEvent(2);
  EXPECT_CALL(*(this->encoder), update())
    .RetiresOnSaturation();
  this->interface->update();
  EXPECT_FALSE(this->interface->getCalibrationMode());
}

TEST_F(InterfaceTestSuite, ignore_master_write_to_readonly_registers) {
  // TODO: Put test here.
}

TEST_F(InterfaceTestSuite, master_write_to_start_end_calibration) {
  // ::testing::InSequence seq;
  // // master writes data to slave to start calibration
  // this->interface->setCalibrationMode(0);
  // EXPECT_FALSE(this->interface->getBusyStatus());
  // EXPECT_FALSE(this->interface->getCalibrationMode());
  // EXPECT_FALSE(this->interface->getCalibratedStatus());
  // uint8_t calibration_start_data[2] = {0x04, 0x04};
  // for(int i = 0; i < 2; i++ ) {
  //   EXPECT_CALL(*(this->twoWire), read())
  //     .WillOnce(::testing::Return(calibration_start_data[i]))
  //     .RetiresOnSaturation();
  // }
  // this->interface->receiveEvent(2);

  // // slave processes data from master
  // EXPECT_CALL(*(this->encoder), update())
  //   .RetiresOnSaturation();
  // this->interface->update();

  // // slave starts calibration
  // EXPECT_CALL(*(this->encoder), startCalibration());
  // EXPECT_CALL(*(this->encoder), update())
  //   .RetiresOnSaturation();
  // this->interface->update();
  // EXPECT_TRUE(this->interface->getCalibrationMode());

  // // slave does calibration until stopped
  // EXPECT_CALL(*(this->encoder), update())
  //   .RetiresOnSaturation();
  // this->interface->update();
  // EXPECT_TRUE(this->interface->getCalibrationMode());

  // // master writes data to slave to stop calibration
  // uint8_t calibration_stop_data[2] = {0x04, 0x01};
  // for(int i = 0; i < 2; i++ ) {
  //   EXPECT_CALL(*(this->twoWire), read())
  //     .WillOnce(::testing::Return(calibration_stop_data[i]))
  //     .RetiresOnSaturation();
  // }
  // this->interface->receiveEvent(2);

  // // slave processes data from master
  // EXPECT_CALL(*(this->encoder), update())
  //   .RetiresOnSaturation();
  // this->interface->update();

  // // slave stops calibration
  // EXPECT_CALL(*(this->encoder), stopCalibration())
  //   .RetiresOnSaturation();
  // EXPECT_CALL(*(this->encoder), update())
  //   .RetiresOnSaturation();
  // this->interface->update();
  // EXPECT_FALSE(this->interface->getCalibrationMode());
  // EXPECT_TRUE(this->interface->getCalibratedStatus());
}

TEST_F(InterfaceTestSuite, master_write_reset_home) {
  ::testing::InSequence seq;
  
  // master writes data to slave
  uint8_t data[2] = {0x04, (0x02+0x01)};
  for(int i = 0; i < 2; i++ ) {
    EXPECT_CALL(*(this->twoWire), read())
      .WillOnce(::testing::Return(data[i]))
      .RetiresOnSaturation();
  }
  this->interface->receiveEvent(2);

  // slave processes data from master
  EXPECT_CALL(*(this->encoder), resetPosition())
    .RetiresOnSaturation();
  EXPECT_CALL(*(this->encoder), update())
    .RetiresOnSaturation();
  this->interface->update();
  EXPECT_FALSE(this->interface->getResetHomeMode());
}

TEST_F(InterfaceTestSuite, master_read_from_start_point) {
  // put slave in the calibration mode
  this->interface->setCalibrationMode(1);

  // tell slave the master wants to read starting from the MODE register
  EXPECT_CALL(*(this->twoWire), read())
    .WillOnce(::testing::Return(MODE_INDEX));
  this->interface->receiveEvent(1);

  // call update which will start calibration
  EXPECT_CALL(*(this->encoder), startCalibration());
  EXPECT_CALL(*(this->encoder), update());
  this->interface->update();

  // tell slave to send master data (which should be the mode)
  uint8_t mode_value;
  EXPECT_CALL(*(this->twoWire), write(::testing::_, ::testing::_))
    .WillOnce(::testing::SaveArgPointee<0>(&mode_value));
  this->interface->requestEvent();
  // mode tracking (0x01) + mode calibrating (0x04) = 0x05
  EXPECT_EQ(CALIBRATION_MODE+TRACKING_MODE, mode_value);
}

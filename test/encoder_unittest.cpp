#include <gtest/gtest.h>
#include <vector>
#include <bitset>
#include <string>
#include "interface.h"
#include "encoder.cpp"
#include "Arduino.h"

using namespace JTIncrementalEncoder;

class EncoderTestSuite : public testing::Test {
  void SetUp() {
  }
  void TearDown() {
  }
};

TEST_F(EncoderTestSuite, isort_returns_sorted_for_sorted) {
  int arr[] = { 1, 2, 3, 4, 5 };
  int arr_sorted[] = { 1, 2, 3, 4, 5 };
  isort(arr, 5);
  for( int i = 0; i < 5; i++ ) 
    EXPECT_EQ(arr_sorted[i], arr[i]);
}

TEST_F(EncoderTestSuite, isort_returns_sorted_for_reverse_sorted) {
  int arr[] = { 5, 4, 3, 2, 1 };
  int arr_sorted[] = { 1, 2, 3, 4, 5 };
  isort(arr, 5);
  for( int i = 0; i < 5; i++ ) 
    EXPECT_EQ(arr_sorted[i], arr[i]);
}

TEST_F(EncoderTestSuite, isort_returns_sorted_for_random) {
  int arr[] = { 4, 5, 3, 2, 1 };
  int arr_sorted[] = { 1, 2, 3, 4, 5 };
  isort(arr, 5);
  for( int i = 0; i < 5; i++ ) 
    EXPECT_EQ(arr_sorted[i], arr[i]);
}

TEST_F(EncoderTestSuite, isort_returns_sorted_one_out_of_place) {
  int arr[] = { 2, 3, 4, 5, 1 };
  int arr_sorted[] = { 1, 2, 3, 4, 5 };
  isort(arr, 5);
  for( int i = 0; i < 5; i++ ) 
    EXPECT_EQ(arr_sorted[i], arr[i]);
}

TEST_F(EncoderTestSuite, getDirection_CLOCKWISE) {
  EXPECT_EQ(CLOCKWISE, getDirection(0x12)); // 00010010
  EXPECT_EQ(CLOCKWISE, getDirection(0x79)); // 01111001
  EXPECT_EQ(CLOCKWISE, getDirection(0x8D)); // 10001101
  EXPECT_EQ(CLOCKWISE, getDirection(0xE6)); // 11100110
}

TEST_F(EncoderTestSuite, getDirection_COUNTERCLOCKWISE) {
  EXPECT_EQ(COUNTERCLOCKWISE, getDirection(0xD2)); // 11010010
  EXPECT_EQ(COUNTERCLOCKWISE, getDirection(0x49)); // 01001001
  EXPECT_EQ(COUNTERCLOCKWISE, getDirection(0xBD)); // 10111101
  EXPECT_EQ(COUNTERCLOCKWISE, getDirection(0xD6)); // 11010110
}

TEST_F(EncoderTestSuite, sample_all_same) {
  std::vector<int> values;
  for( int i = 0; i < 5; i++ ) {
    values.push_back((uint8_t)128);
  }
  pinMock.set_analog_value_sequence(5, values);
  uint8_t rvalue = sample((uint8_t)5);
  EXPECT_EQ(128, rvalue);
}

TEST_F(EncoderTestSuite, sample_varied) {
  std::vector<int> values;
  values.push_back(182);
  values.push_back(2);
  values.push_back(128);
  values.push_back(4);
  values.push_back(243);
  pinMock.set_analog_value_sequence(5, values);
  uint8_t rvalue = sample((uint8_t)5);
  EXPECT_EQ(128, rvalue);
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

  EXPECT_EQ(std::bitset<8>(std::string("10110100")), std::bitset<8>(encoder_state));
  EXPECT_EQ(5, index);

  // check read low value
  updateChannelAEncoderState(channel, encoder_state, index);

  EXPECT_EQ(std::bitset<8>(std::string("11100101")), std::bitset<8>(encoder_state));
  EXPECT_EQ(6, index);

  // check that the index doesn't overflow into the encoder values
  index = 16;
  encoder_state_bitset = std::bitset<8>(std::string("00101111"));
  encoder_state = encoder_state_bitset.to_ulong();

  updateChannelAEncoderState(channel, encoder_state, index);

  EXPECT_EQ(std::bitset<8>(std::string("10100000")), std::bitset<8>(encoder_state));
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

  EXPECT_EQ(std::bitset<8>(std::string("01110100")), std::bitset<8>(encoder_state));
  EXPECT_EQ(5, index);

  // check read low value
  updateChannelBEncoderState(channel, encoder_state, index);

  EXPECT_EQ(std::bitset<8>(std::string("11010101")), std::bitset<8>(encoder_state));
  EXPECT_EQ(6, index);

  // check that the index doesn't overflow into the encoder values
  index = 16;
  encoder_state_bitset = std::bitset<8>(std::string("00101111"));
  encoder_state = encoder_state_bitset.to_ulong();

  updateChannelBEncoderState(channel, encoder_state, index);

  EXPECT_EQ(std::bitset<8>(std::string("10000000")), std::bitset<8>(encoder_state));
  EXPECT_EQ(17, index);
}

TEST_F(EncoderTestSuite, runCalibration) {
  int analog_values_a_arr[100] = { 573, 848, 708, 560, 801, 401, 701, 276, 395, 699, 510, 481, 419, 210, 130, 37, 742, 
    926, 602, 160, 952, 675, 410, 58, 724, 566, 899, 416, 240, 692, 763, 253, 571, 741, 432, 333, 356, 847, 959, 307, 
    847, 744, 730, 264, 211, 959, 977, 165, 383, 556, 943, 305, 714, 604, 397, 483, 6, 187, 150, 532, 591, 366, 450, 
    108, 499, 928, 556, 526, 554, 133, 821, 448, 922, 530, 118, 516, 906, 146, 126, 496, 709, 823, 747, 332, 83, 992, 
    243, 143, 794, 73, 625, 907, 665, 351, 813, 867, 984, 425, 337, 116 };

  int analog_values_b_arr[100] = { 171, 157, 917, 278, 320, 402, 576, 286, 718, 200, 189, 908, 364, 527, 179, 844, 
    996, 550, 588, 47, 373, 893, 705, 86, 380, 698, 619, 436, 993, 258, 820, 397, 740, 862, 160, 864, 656, 924, 764, 
    936, 532, 288, 484, 343, 145, 154, 730, 914, 23, 972, 67, 238, 289, 178, 551, 868, 501, 882, 349, 709, 747, 145, 
    954, 203, 707, 160, 953, 39, 25, 168, 716, 712, 210, 527, 417, 490, 444, 718, 68, 936, 117, 428, 285, 44, 443, 
    528, 117, 600, 173, 210, 834, 640, 237, 32, 539, 966, 979, 219, 316, 931 };

  std::vector<int> analog_a_values;
  std::vector<int> analog_b_values;
  std::vector<uint8_t> is_calibration_active_values;

  EncoderChannelPair channels;

  channels.a.rawInputPin = 5;
  channels.b.rawInputPin = 6;

  for( int i = 0; i < 100; i++ ) {
    analog_a_values.push_back(analog_values_a_arr[i]);
  }
  pinMock.set_analog_value_sequence(5, analog_a_values);

  for( int i = 0; i < 100; i++ ) {
    analog_b_values.push_back(analog_values_b_arr[i]);
  }
  pinMock.set_analog_value_sequence(6, analog_b_values);

  for( int i = 0; i < 20; i++ ) {
    is_calibration_active_values.push_back(1);
  }
  is_calibration_active_values.push_back(0);
  EncoderInterface.set_calibration_mode_sequence(is_calibration_active_values);

  runCalibration(channels);

  EXPECT_EQ(229, channels.a.maxValue);
  EXPECT_EQ(44, channels.a.minValue);
  EXPECT_EQ((229+44) / 2, channels.a.average);

  EXPECT_EQ(200, channels.b.maxValue);
  EXPECT_EQ(66, channels.b.minValue);
  EXPECT_EQ((200+66) / 2, channels.b.average);
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

TEST_F(EncoderTestSuite, saveSettings_loadSettings) {
  // create some encoder settings
  EncoderChannelPair pair;
  pair.a.minValue = 5;
  pair.a.maxValue = 120;
  pair.a.average = 70;
  pair.b.minValue = 23;
  pair.b.maxValue = 1022;
  pair.b.average = 571;

  // save the encoder pair
  int addr = 5;
  saveSettings(pair.a, addr);
  saveSettings(pair.b, addr);

  // load the encoder pair
  EncoderChannelPair loadedPair;
  addr = 5;
  loadSettings(loadedPair.a, addr);
  loadSettings(loadedPair.b, addr);

  // is what we loaded the same as what we saved?
  EXPECT_EQ(pair.a.minValue, loadedPair.a.minValue);
  EXPECT_EQ(pair.a.maxValue, loadedPair.a.maxValue);
  EXPECT_EQ(pair.a.average, loadedPair.a.average);
  EXPECT_EQ(pair.b.minValue, loadedPair.b.minValue);
  EXPECT_EQ(pair.b.maxValue, loadedPair.b.maxValue);
  EXPECT_EQ(pair.b.average, loadedPair.b.average);
}

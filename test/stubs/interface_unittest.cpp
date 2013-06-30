#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include <bitset>
#include <string>
#include "i2c_interface.cpp"
#include "Arduino.h"

class InterfaceTestSuite : public ::testing::Test {
public:
  void SetUp() {
  }

  void TearDown() {
  }
};

TEST_F(InterfaceTestSuite, empty_test) {
  
}

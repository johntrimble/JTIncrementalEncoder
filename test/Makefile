# Google Test root directory
GTEST_DIR = googletest
GMOCK_DIR = googlemock

# where sources to test are
USER_DIR = ..
# where test sources are
TEST_DIR = $(USER_DIR)/test

CPPFLAGS += -isystem $(GTEST_DIR)/include -isystem $(GMOCK_DIR)/include
CXXFLAGS += -g -Wall -Wextra -pthread

# All tests produced by this Makefile.
TESTS = encoder_unittest

# All Google Test headers.
GTEST_HEADERS = $(GTEST_DIR)/include/gtest/*.h \
                $(GTEST_DIR)/include/gtest/internal/*.h
GMOCK_HEADERS = $(GMOCK_DIR)/include/gmock/*.h \
                $(GMOCK_DIR)/include/gmock/internal/*.h \
                $(GTEST_HEADERS)

# House-keeping build targets.
all : $(TESTS)

clean :
	rm -f $(TESTS) gmock.a gmock_main.a *.o

# Build Google Test
GTEST_SRCS_ = $(GTEST_DIR)/src/*.cc $(GTEST_DIR)/src/*.h $(GTEST_HEADERS)
GMOCK_SRCS_ = $(GMOCK_DIR)/src/*.cc $(GMOCK_HEADERS)

gtest-all.o : $(GTEST_SRCS_)
	$(CXX) $(CPPFLAGS) -I$(GTEST_DIR) -I$(GMOCK_DIR) $(CXXFLAGS) \
            -c $(GTEST_DIR)/src/gtest-all.cc

gmock-all.o : $(GMOCK_SRCS_)
	$(CXX) $(CPPFLAGS) -I$(GTEST_DIR) -I$(GMOCK_DIR) $(CXXFLAGS) \
            -c $(GMOCK_DIR)/src/gmock-all.cc

gmock_main.o : $(GMOCK_SRCS_)
	$(CXX) $(CPPFLAGS) -I$(GTEST_DIR) -I$(GMOCK_DIR) $(CXXFLAGS) \
            -c $(GMOCK_DIR)/src/gmock_main.cc

gmock.a : gmock-all.o gtest-all.o
	$(AR) $(ARFLAGS) $@ $^

gmock_main.a : gmock-all.o gtest-all.o gmock_main.o
	$(AR) $(ARFLAGS) $@ $^

# Build mocks
$(TEST_DIR)/Arduino.o : $(TEST_DIR)/mocks/Arduino.cpp $(TEST_DIR)/mocks/Arduino.h 
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $(TEST_DIR)/mocks/Arduino.cpp

$(TEST_DIR)/EEPROM.o : $(TEST_DIR)/mocks/EEPROM.cpp $(TEST_DIR)/mocks/EEPROM.h
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $(TEST_DIR)/mocks/EEPROM.cpp

$(TEST_DIR)/Wire.o : $(TEST_DIR)/mocks/Wire.cpp $(TEST_DIR)/mocks/Wire.h
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $(TEST_DIR)/mocks/Wire.cpp

$(TEST_DIR)/interface_stub.o : $(TEST_DIR)/mocks/interface_stub.cpp $(TEST_DIR)/mocks/interface.h
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -I$(TEST_DIR)/mocks -I$(USER_DIR) -I$(TEST_DIR) -c $(TEST_DIR)/mocks/interface_stub.cpp

# Build tests
encoder_unittest.o : $(TEST_DIR)/encoder_unittest.cpp \
					 $(TEST_DIR)/Arduino.o \
					 $(TEST_DIR)/EEPROM.o \
					 $(TEST_DIR)/Wire.o \
					 $(TEST_DIR)/interface_stub.o \
                     $(GTEST_HEADERS)
	$(CXX) $(CPPFLAGS) -I$(TEST_DIR)/mocks -I$(USER_DIR) -I$(TEST_DIR) $(CXXFLAGS) -c $(TEST_DIR)/encoder_unittest.cpp

encoder_unittest : encoder_unittest.o $(TEST_DIR)/Arduino.o $(TEST_DIR)/EEPROM.o $(TEST_DIR)/Wire.o $(TEST_DIR)/interface_stub.o gmock_main.a
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -lpthread $^ -o $@

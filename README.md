# The JT Incremental Encoder
This is the Arduino source code for an optical rotary incremental encoder I built as a sort of hobby project. It provides 2-channel Quadrature output, as well as an I2C interface for calibration and reading position information.

## Dependencies
- Arduino IDE 1.5 or greater
- MCP4xxx library, available [here](https://github.com/johntrimble/MCP42xxx)
- g++ 4.2 or greater, this is just for the unit tests
- Make
- Git 1.6.5 or greater

## Building
This project uses submodules, so you'll want to do the following when cloning the repository:

```
git clone --recursive git@github.com:johntrimble/JTIncrementalEncoder.git
```

From there, just open the `JTIncrementalEncoder.ino` in the Arduino IDE and build it just like any other Arduino project.

## Unit Testing
Arduino doesn't have much in the way of support for unit testing code. Consequently, in the test folder there is a Makefile for building some native binaries to test the code. To run them, execute the following from the project root:

```
cd test
make
./interface_unittest && ./encoder_unittest
```

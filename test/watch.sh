#!/usr/bin/env bash
watchmedo shell-command --wait --recursive --patterns '*.cpp;*.h;*.ino' --command "make clean && make all && ./encoder_unittest && ./interface_unittest" ..

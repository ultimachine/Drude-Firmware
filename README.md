# Drude Frimware

[![Build Status](https://travis-ci.org/ultimachine/Drude-Firmware.svg?branch=master)](https://travis-ci.org/ultimachine/Drude-Firmware)

### Install AVR Toolchain
sudo apt-get install gcc-avr binutils-avr gdb-avr avr-libc avrdude

## Clone
git clone https://github.com/ultimachine/Drude-Firmware.git

cd Drude-Firmware

git submodule init

git submodule update

## Compile
make

## Upload
enter DFU Mode by holding button S2 and momentarily press button S1

Verify DFU Mode:

Run: lsusb | grep 2ff4

Output: Bus 003 Device 008: ID 03eb:2ff4 Atmel Corp.

Run:

dfu-programmer atmega32u4 erase

dfu-programmer atmega32u4 flash VirtualSerialDigitizer.hex

dfu-programmer atmega32u4 reset

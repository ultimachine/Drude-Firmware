#!/bin/bash
sudo avrdude -p m32u4 -P /dev/ttyACM0 -c avr109 -Uflash:r:$1:i

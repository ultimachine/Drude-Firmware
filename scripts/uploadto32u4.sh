#!/bin/bash
avrdude -p m32u4 -P usb -c avrispmkII -e -Uflash:w:$1:i

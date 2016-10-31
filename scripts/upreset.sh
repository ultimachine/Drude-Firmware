#!/bin/bash
avrdude -p m32u4 -P usb -c avrispmkII -U lfuse:r:-:i

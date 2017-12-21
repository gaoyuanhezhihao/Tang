#!/usr/bin/sudo bash
socat  PTY,link=/dev/ttyUSB0 PTY,link=/dev/ttyUSB2 & 
socat  PTY,link=/dev/ttyUSB1 PTY,link=/dev/ttyUSB3 &
echo "change ttyUSB owner"
chown hzh /dev/ttyUSB*




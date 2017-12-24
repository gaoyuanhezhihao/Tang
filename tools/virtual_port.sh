#!/usr/bin/sudo bash
socat  PTY,link=/dev/ttyUSB0 PTY,link=/dev/ttyUSB2 & 
socat  PTY,link=/dev/ttyUSB1 PTY,link=/dev/ttyUSB3 &
echo "change ttyUSB owner"
cnt=0
while [ $cnt -lt 3 ]  &&  ( [ ! -e /dev/ttyUSB0 ] || [ ! -e /dev/ttyUSB1 ] || [  ! -e /dev/ttyUSB2 ] || [ ! -e /dev/ttyUSB3 ] )
do
    cnt=`expr $cnt + 1`
    echo $cnt
    echo "USBs are not ready yet, waiting..."
    sleep 1
done
if [ $cnt -ge 3 ]
then
    echo "Error. Wait Too Long"
    exit -1
fi
chown hzh /dev/ttyUSB*




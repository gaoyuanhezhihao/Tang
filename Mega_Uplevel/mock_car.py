# pyserial_chart_receiver.py
# pyserial_sender.py
from __future__ import print_function
import serial
import string
import random
from time import sleep
import sys
def serial_ports():
    """Lists serial ports

    :raises EnvironmentError:
        On unsupported or unknown platforms
    :returns:
        A list of available serial ports
    """
    if 'linux' in sys.platform:
        ports = ['/dev/ttyUSB' + str(i) for i in range(256)]
        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
    elif 'win' in sys.platform:
        ports = ['com' + str(i) for i in range(256)]
        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
    else:
        print("error: unknow system platform")
    return result


if __name__ == '__main__':
    print("serial port available:\n")
    print(serial_ports())
    sPortChoic=raw_input("Input the port to open\n")
    port=serial.Serial(sPortChoic,9600)
    rcv = ''
    send_msg = ''
    while 1:
        rcv = port.read(1)
        if rcv == 'H':
            rcv += port.read(3)
            print('--> ', rcv[0:2],',', ord(rcv[2]), ',', ord(rcv[3]))
            if rcv[1] in ['f', 'd', 'b', 'l', 'r', 's', 'w', 'g']:
                send_msg = rcv[1] + "_ack\r\n"
                port.write(send_msg)
                print('<--', send_msg)
            elif rcv[1]  in ['y', 'z']:
                send_msg = rcv[1] + "_ack\r\n"
                port.write(send_msg)
                print('<--', send_msg)
                print('...waiting turning to completed')
                sleep(2)
                send_msg = rcv[1] + "_ok\r\n"
                port.write(send_msg)
                print('<--', send_msg)

            elif rcv[1] == 'D':
                send_msg = "D_ack\r\n"
                port.write(send_msg)
                print('<--', send_msg)

                print("...waiting going completed")
                sleep(2)

                send_msg = "D_ok\r\n"
                port.write(send_msg)
                print('<--', send_msg)





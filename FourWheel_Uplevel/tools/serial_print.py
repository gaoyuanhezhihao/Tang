
from __future__ import print_function
import serial
from platform import platform
import time


if 'Linux' in platform():
    PORT_PREFIX = '/dev/ttyUSB'
elif 'Windows' in platform():
    PORT_PREFIX = 'com'


class _CONST:

    def __init__(self, name):
        self.name = name

    def __setattr__(self, name, value):
        if name in self.__dict__:
            raise TypeError, "(%s) is const"%(name)
        else:
            self.__dict__[name] = value


const = _CONST("global")
const.split_flag = '\r\n'


class SerialMonitor(object):

    def __init__(self):
        print("serial port available:\n")
        print(self.serial_ports())
        sPortChoic = raw_input("Input the port number to open\n")
        self.port = serial.Serial(PORT_PREFIX+sPortChoic, 9600)
        # self.waiting_turn_ok = ''
        self.tmp_msg = ''
        self.state = 's'
        self.start_turn_time = time.time()
        self.msg_list = []
        self.last_peek_time = 0

    def serial_ports(self):
        ports = [PORT_PREFIX + str(i) for i in range(256)]
        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result

    def rcv_uart_msg(self):
        byte_2_read = self.port.inWaiting()
        if byte_2_read >= 1:
            rcv = self.port.read(byte_2_read)
            self.tmp_msg += rcv
            new_msg_list = self.tmp_msg.split(const.split_flag)
            if len(new_msg_list) > 1:
                self.tmp_msg = new_msg_list[-1]
                self.msg_list += new_msg_list[0:-1]
                return True
        return False

    def loop(self):
        while True:
            self.rcv_uart_msg()
            for pack in self.msg_list:
                print("rcv:%s" % repr(pack))
            self.msg_list = []
if __name__ == '__main__':
    monitor = SerialMonitor()
    monitor.loop()

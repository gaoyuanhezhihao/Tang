# xy_robot.py
'''
This software is for Tang's xy_robot.
'''

import struct
import serial
import Tkinter
import time
import pickle
import pdb
import logging
from platform import platform

SYS_LOGGER_NAME = "xy_robot"

if 'Linux' in platform():
    PORT_PREFIX = '/dev/ttyUSB'
elif 'Windows' in platform():
    PORT_PREFIX = 'com'

class CarAdmin():

    def __init__(self, name):
        if __name__ == "main":
            logger = logging.getLogger(SYS_LOGGER_NAME)
            logger.setLevel(logging.DEBUG)
            # create file handler which logs even debug messages
            fh = logging.FileHandler(SYS_LOGGER_NAME+".log")
            fh.setLevel(logging.DEBUG)
            # create console handler with a higher log level
            ch = logging.StreamHandler()
            ch.setLevel(logging.ERROR)
            # create formatter and add it to the handlers
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            fh.setFormatter(formatter)
            ch.setFormatter(formatter)
        else:
            self.logger = logging.getLogger(SYS_LOGGER_NAME)
            self.name = name
            self.State = 's'
            self.RcvBuffer = []
            self.LastSentOrder = 's'
            self.LastAckTime = 0
            print("serial port available:\n")
            print(self.serial_ports())
            sPortChoic = raw_input("Input the num of port to open\n")
            self.port = serial.Serial(PORT_PREFIX + sPortChoic, 9600)

            self.state = 0
            self.msg_list = []
            self.tmp_msg = ''


    def get_mv_time(self):
        str_time = self.time_entry.get()
        if 0 == len(str_time):
            print "Error! time is empty"
            return None
        elif str_time.isdigit():
            time = int(str_time)
            return time
        else:
            print "Error! invalid time format(should be a digit!)"
            return None


    def mv2x_org(self):
        self.State = 'x'
        self.Send_Direct_Order(order='x')

    def mv2y_org(self):
        self.State = 'y'
        self.Send_Direct_Order(order='y')

    def Left(self):
        self.State = 'l'
        time = self.get_mv_time()
        if time is not None:
            self.Send_Direct_Order(order='l', data1=time/256,
                              data2=time%256)

    def Right(self):
        self.state = 'r'
        time = self.get_mv_time()
        if time is not None:
            self.Send_Direct_Order(order='r', data1=time/256,
                              data2=time%256)

    def Up(self):
        self.state = 'u'
        time = self.get_mv_time()
        if time is not None:
            self.Send_Direct_Order(order='u', data1=time/256,
                              data2=time%256)

    def Down(self):
        self.state = 'd'
        time = self.get_mv_time()
        if time is not None:
            self.Send_Direct_Order(order='d', data1=time/256,
                              data2=time%256)

    def Stop(self):
        self.state = 's'
        self.Send_Direct_Order(order='s')

    # def update_pwm(self):
        # self.pwm = int(self.time_entry.get())
        # print "Your pwm:%s" % self.time_entry.get()
        # self.Send_Direct_Order(order='p', data1=self.pwm/256,
                               # data2=self.pwm % 256)

    # def Send_Secret_Order(self, PWM_left=None, PWM_right=None, order=None,
                          # data1=None, data2=None):
        # if(order is None):
            # msg = '$DCR:' + str(PWM_left) + str(-500) + \
                    # ',' + str(PWM_right) + str(-500) + '!'
            # self.port.write(msg)
            # if __name__ == "__main__":
                # print msg, '\n'
                # self.logger.info("Send_Direct_Order:"+msg)
            # else:
                # if data1 is None or data2 is None:
                    # data1 = 0x00
                    # data2 = 0x00
                    # self.port.write(['H'])
                    # self.port.write([order])
                    # self.port.write([data1])
                    # self.port.write([data2])
                    # self.logger.info("Send_Secret_Order:"+order)
                    # return 0



    def Send_Direct_Order(self, order,
                          data1=0xff, data2=0xff):
        msg = 'H'
        msg += order
        msg += struct.pack('!B', data1)
        msg += struct.pack('!B',data2)
        self.port.write(msg)
        # print order
        # self.port.write('H')
        # self.port.write(order)
        # self.port.write(data1)
        # self.port.write(data2)

        self.send_msg_time = time.time()

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



    def check_last_send(self):
        if time.time() - self.send_msg_time > 0.7 and self.last_order != 0:
            self.Send_Direct_Order(order=self.last_order,
                                   data1=self.last_data1,
                                   data2=self.last_data2)
            self.send_msg_time = time.time()

    def rcv_uart_msg(self):
        byte_2_read = self.port.inWaiting()
        if byte_2_read >= 1:
            rcv = self.port.read(byte_2_read)
            self.tmp_msg += rcv
            self.msg_list = self.tmp_msg.split('\n')
            if len(self.msg_list) > 1:
                for msg in self.msg_list:
                    print msg
                self.tmp_msg = ''
                self.msg_list = []

    def Run(self):
        self.calibra_panel = Tkinter.Tk()
        self.Forward_Button = Tkinter.Button(
            self.calibra_panel, text="Up",
            command=self.Up)
        self.Forward_Button.pack()

        self.Backward_button = Tkinter.Button(
            self.calibra_panel, text="Down",
            command=self.Down)
        self.Backward_button.pack()

        self.Left_button = Tkinter.Button(
            self.calibra_panel, text="Left",
            command=self.Left)
        self.Left_button.pack()

        self.Right_button = Tkinter.Button(
            self.calibra_panel, text="Right",
            command=self.Right)
        self.Right_button.pack()

        self.x_org_button = Tkinter.Button(
            self.calibra_panel, text="X Origin",
            command=self.mv2x_org)
        self.x_org_button.pack()

        self.y_org_button = Tkinter.Button(
            self.calibra_panel, text="Y Origin",
            command=self.mv2y_org)
        self.y_org_button.pack()

        self.Stop_button = Tkinter.Button(
            self.calibra_panel, text="stop", command=self.Stop)
        self.Stop_button.pack()



        self.time_set = Tkinter.StringVar()
        self.time_entry = Tkinter.Entry(
            self.calibra_panel, textvariable=self.time_set)
        self.time_entry.pack()

        while True:
            self.calibra_panel.update()
            # self.check_last_send()
            self.rcv_uart_msg()

if __name__ == '__main__':
    Admin = CarAdmin('Car')
    Admin.Run()

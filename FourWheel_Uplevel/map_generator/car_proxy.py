# car_proxy.py

from __future__ import print_function
from platform import platform
import logging
import serial
import struct
import time

from const_var import const

if 'Linux' in platform():
    PORT_PREFIX = '/dev/ttyS'
elif 'Windows' in platform():
    PORT_PREFIX = 'com'
const.retry_limit = 2  # if failed after 2 retries, Raise error.
const.wait_time_limit = 0.5  # wait no more than 500ms when receiving reply.
const.pulses_per_degree = 1720/90
const.pulses_per_cm = 65.57
const.split_flag = '\r\n'
const.wait_turn_limit = 3


class CarProxy():

    def __init__(self, logger):
        self.logger = logger
        # init serial port
        print("serial port available:\n")
        print(self.serial_ports())
        sPortChoic = raw_input("Input the port number to open\n")
        self.port = serial.Serial(PORT_PREFIX+sPortChoic, 9600)
        # self.waiting_turn_ok = ''
        self.tmp_msg = ''
        self.state = 's'
        self.start_turn_time = time.time()

    def set_map_std_dist(self, std_dist):
        self.map_std_dist = std_dist
        self.Send_Direct_Order(order='M', data1=std_dist/256,
                               data2=std_dist % 256)

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

    def retry_last_order(self):
        self.Send_Direct_Order(order=self.last_order, data1=self.last_data1,
                               data2=self.last_data2)

    def Send_Direct_Order(self, order=None, data1=None, data2=None):
        if data1 is None or data2 is None:
            data1 = 0x00
            data2 = 0x00
        msg = 'H'
        msg += order
        msg += struct.pack('!B', data1)
        msg += struct.pack('!B', data2)
        self.port.write(msg)
        self.last_order = order
        self.last_data1 = data1
        self.last_data2 = data2
        self.right_ack = self.last_order + '_ack'
        self.send_msg_time = time.time()
        self.logger.info("Send_Direct_Order:"+msg)

    def update_pwm(self):
        self.pwm = int(self.pwm_entry.get())
        self.Send_Direct_Order(order='p', data1=self.pwm/256,
                               data2=self.pwm % 256)

    def step_left(self, angle):
        steps = int(angle * const.pulses_per_degree)
        self.Send_Direct_Order(order='l', data1=steps/256,
                               data2=steps % 256)
        # self.waiting_turn_ok = 'l'
        self.start_turn_time = time.time()
        self.state = 'l'

    def step_right(self, angle):
        steps = int(angle * const.pulses_per_degree)
        self.Send_Direct_Order(order='r', data1=steps/256,
                               data2=steps % 256)
        # self.waiting_turn_ok = 'r'
        self.start_turn_time = time.time()
        self.state = 'r'

    def change_std_len(self):
        std_len = int(self.std_len_entry.get())
        self.Send_Direct_Order(order='d', data1=std_len/256,
                               data2=std_len % 256)
        self.state = 's'

    def go_dist(self, dist_in_cm):
        steps = dist_in_cm * const.pulses_per_cm
        self.Send_Direct_Order(order='D', data1=steps/256,
                               data2=steps % 256)
        self.state = 'f'

    def go_backward(self, dist_in_cm):
        steps = dist_in_cm * const.pulses_per_cm
        self.Send_Direct_Order(order='B', data1=steps/256,
                               data2=steps % 256)
        self.state = 'b'

    # def TurnLeft(self):
        # self.State = 'l'
        # self.Send_Direct_Order(order='l')

    # def TurnRight(self):
        # self.state = 'r'
        # self.Send_Direct_Order(order='r')

    def Forward(self):
        self.state = 'f'
        self.Send_Direct_Order(order='f')
        self.verify_or_retry('f_ack')

    def Backward(self):
        self.Send_Direct_Order(order='b')
        self.verify_or_retry('b_ack')
        self.state = 'b'

    def Stop(self):
        self.Send_Direct_Order(order='s')
        self.state = 's'

    def map_forward(self):
        self.Send_Direct_Order('F')
        self.verify_or_retry('F_ack')
        self.state = 'f'

    def map_backward(self):
        self.Send_Direct_Order('B')
        self.verify_or_retry('B_ack')
        self.state = 'b'

    def map_stop(self):
        self.Send_Direct_Order('S')
        last_map_dist = self.verify_map_stop()
        self.logger.info("last map dist:%s" % last_map_dist)
        self.state = 's'
        return last_map_dist

    def verify_or_retry(self, ack_msg):
        retry_count = 0
        while retry_count <= const.retry_limit:
            retry_count += 1
            flag, msg = self.rcv_ack(len(ack_msg))
            if ack_msg in msg:
                return True
        # connection failed.
        raise Exception("Car Error", "RETRY TOO TIMES")

    def rcv_ack(self, msg_len):
        msg_len += len(const.split_flag)
        start_time = time.time()
        bytes_waiting = 0
        while bytes_waiting < msg_len:
            if time.time() - start_time > const.wait_time_limit:
                return False, "wait too long"
            bytes_waiting = self.port.inWaiting()
        rcv = self.port.read(bytes_waiting)
        return True, rcv

    def verify_map_stop(self):
        retry_count = 0
        while True:
            ret, map_dist = self.rcv_map_dist()
            if ret is True:
                return map_dist
            else:
                retry_count += 1
                if retry_count > const.retry_limit:
                    raise Exception("Car Error", "RETRY TOO TIMES")
                self.logger.warning("verify_map_stop: retry again")
                self.retry_last_order()

    def check_turn_ack(self):
        if self.state in ['l', 'r']:
            '''car is in turning state.'''
            ok_reply = self.waiting_turn_ok + '_ok'
            msg = self.rcv_uart_msg()
            if msg is not None:
                for token in msg:
                    if ok_reply in token:
                        # got right reply.
                        # self.waiting_turn_ok = ''
                        self.state = 's'
                        self.logger.info("get reply:" + token)
                        return True
            # check if waits too long?
            if time.time() - self.start_turn_time > const.wait_turn_limit:
                self.peek_state()

    def peek_state(self):
        self.Send_Direct_Order(order='Q')
        self.state = self.get_car_stae()

    def get_car_stae(self):
        retry_count = 0
        good_reply = "Q_ackx"
        while retry_count <= const.retry_limit:
            retry_count += 1
            flag, msg = self.rcv_ack(len(good_reply))
            if flag and "Q_ack" in msg:
                return msg[5]
        # connection failed.
        raise Exception("Car Error", "RETRY TOO TIMES")

    def routines(self):
        self.check_turn_ack()

    def rcv_map_dist(self, wait_time_limit=const.wait_time_limit):
        '''
        reply: "S" + byte1 + byte0+ end_flag
        '''
        byte_2_read = 0
        start_time = time.time()
        while byte_2_read < 5:
            if time.time() - start_time > wait_time_limit:
                return False, "out of time"
            byte_2_read = self.port.inWaiting()
        # got enough bytes to read.
        rcv = self.port.read(byte_2_read)
        token = rcv.split(const.split_flag)[0]
        if token[0] == 'S':
            byte1 = ord(token[1])
            byte0 = ord(token[2])
            return True, byte1 * 256 + byte0
        else:
            return False, "wrong ack"

    def rcv_uart_msg(self):
        byte_2_read = self.port.inWaiting()
        if byte_2_read >= 1:
            rcv = self.port.read(byte_2_read)
            self.tmp_msg += rcv
            self.msg_list = self.tmp_msg.split(const.split_flag)
            if len(self.msg_list) > 1:
                self.tmp_msg = self.msg_list[-1]
                return self.msg_list[0:-1]
        return None

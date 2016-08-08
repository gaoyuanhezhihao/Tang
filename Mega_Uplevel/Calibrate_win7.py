# Calibrate_win7.py
'''
This software is for Tang's car.
'''
import struct
import serial
import Tkinter
import time
import pickle
import pdb
import logging

SYS_LOGGER_NAME = "adaptor"

class CarAdmin():

    def __init__(self, name):
        if __name__ == "main":
            logger = logging.getLogger('Calibrate')
            logger.setLevel(logging.DEBUG)
            # create file handler which logs even debug messages
            fh = logging.FileHandler('Calibrate.log')
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
            sPortChoic = raw_input("Input the port to open\n")
            self.port = serial.Serial(sPortChoic, 9600)
            self.SentOrderRecord = 0
            self.last_direct_order = 0
            self.LeftRotate = 0
            self.RightRotate = 0
            self.calibra_state = 0
            self.last_order_time = 0
            self.pwm_degree = 1
            self.MAX_PWM_DEGREE = 4
            self.pwm_preset = []
            self.debug_left_pwm_buf = 0
            self.debug_right_rotate_buf = 0
            self.debug_left_rotate_buf = 0
            self.debug_right_rotate_buf = 0
            self.debug_left_pwm_buf = 0
            self.debug_right_pwm_buf = 0
            self.cycle_pair = {'f': 'b', 'b': 'f'}
            self.cycle_state = 0
            self.last_cycle_time = 0
            self.state = 0
            self.last_order = 0
            self.send_msg_time = 0
            self.msg_list = []
            self.tmp_msg = ''

    def TurnLeft(self):
        self.State = 'l'
        self.Send_Direct_Order(order='l')

    def TurnRight(self):
        self.state = 'r'
        self.Send_Direct_Order(order='r')

    def Forward(self):
        self.state = 'f'
        self.Send_Direct_Order(order='f')

    def Backward(self):
        self.state = 'b'
        self.Send_Direct_Order(order='b')

    def Stop(self):
        self.state = 's'
        self.Send_Direct_Order(order='s')

    def cycle(self):
        self.state = 'cycle'
        self.Send_Direct_Order(order='f')
        self.last_cycle_time = time.time()
        self.cycle_state = 'f'

    def update_pwm(self):
        self.pwm = int(self.pwm_entry.get())
        print "Your pwm:%s" % self.pwm_entry.get()
        self.Send_Direct_Order(order='p', data1=self.pwm/256,
                               data2=self.pwm % 256)

    def Send_Secret_Order(self, PWM_left=None, PWM_right=None, order=None,
                          data1=None, data2=None):
        if(order is None):
            msg = '$DCR:' + str(PWM_left) + str(-500) + \
                    ',' + str(PWM_right) + str(-500) + '!'
            self.port.write(msg)
            if __name__ == "__main__":
                print msg, '\n'
                self.logger.info("Send_Direct_Order:"+msg)
            else:
                if data1 is None or data2 is None:
                    data1 = 0x00
                    data2 = 0x00
                    self.port.write(['H'])
                    self.port.write([order])
                    self.port.write([data1])
                    self.port.write([data2])
                    self.logger.info("Send_Secret_Order:"+order)
                    return 0

    def step_left(self):
        self.steps = int(self.steps_entry.get())
        print "steps:%s" % self.steps_entry.get()
        self.Send_Direct_Order(order='y', data1=self.steps/256,
                               data2=self.steps % 256)

    def step_right(self):
        self.steps = int(self.steps_entry.get())
        print "steps:%s" % self.steps_entry.get()
        self.Send_Direct_Order(order='z', data1=self.steps/256,
                               data2=self.steps % 256)

    def change_std_len(self):
        std_len = int(self.std_len_entry.get())
        self.Send_Direct_Order(order = 'd', data1=std_len/256,
                              data2=std_len%256)
        
    def go_dist(self):
        dist = int(self.dist_in_std_len_entry.get())
        self.Send_Direct_Order(order = 'D', data1=dist/256,
                              data2 = dist%256)
    def go_backward(self):
        dist = int(self.dist_in_std_len_entry.get())
        self.Send_Direct_Order(order = 'B', data1=dist/256,
                              data2 = dist%256)


    def Send_Direct_Order(self, PWM_left=None, PWM_right=None, order=None,
                          data1=None, data2=None):
        if data1 is None or data2 is None:
            data1 = 0x00
            data2 = 0x00
        msg = 'H'
        msg += order
        msg += struct.pack('!B', data1)
        msg += struct.pack('!B',data2)
        self.port.write(msg)
        print order
        #self.port.write('H')
        #self.port.write(order)
        #self.port.write(data1)
        #self.port.write(data2)
        self.last_order = order
        self.last_data1 = data1
        self.last_data2 = data2
        self.right_ack = self.last_order + '_ack'
        self.send_msg_time = time.time()
        return 0
    def serial_ports(self):
        ports = ['com' + str(i) for i in range(256)]
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
        if self.last_order == 0:
            return 0
        byte_2_read = self.port.inWaiting()
        if byte_2_read >= 1:
            rcv = self.port.read(byte_2_read)
            self.tmp_msg += rcv
            self.msg_list = self.tmp_msg.split('\n')
            if len(self.msg_list) > 1:
                self.tmp_msg = self.msg_list[-1]
                for m in self.msg_list[:-1]:
                    # pdb.set_trace()
                    if self.right_ack in m:
                        self.last_order = 0
                    print m
            
    def Run(self):
        self.calibra_panel = Tkinter.Tk()
        self.Forward_Button = Tkinter.Button(
            self.calibra_panel, text="Forward",
            command=self.Forward)
        self.Forward_Button.pack()

        self.Backward_button = Tkinter.Button(
            self.calibra_panel, text="Backward",
            command=self.Backward)
        self.Backward_button.pack()

        self.Left_button = Tkinter.Button(
            self.calibra_panel, text="Left",
            command=self.TurnLeft)
        self.Left_button.pack()

        self.Right_button = Tkinter.Button(
            self.calibra_panel, text="Right",
            command=self.TurnRight)
        self.Right_button.pack()

        self.Stop_button = Tkinter.Button(
            self.calibra_panel, text="stop", command=self.Stop)
        self.Stop_button.pack()

        self.Cycle_button = Tkinter.Button(
            self.calibra_panel, text="cycle", command=self.cycle)
        self.Cycle_button.pack()

        self.pwm_set = Tkinter.StringVar()
        self.pwm_entry = Tkinter.Entry(
            self.calibra_panel, textvariable=self.pwm_set)
        self.pwm_entry.pack()

        self.pwm_update_btn = Tkinter.Button(
            self.calibra_panel, text="pwm change", command=self.update_pwm)
        self.pwm_update_btn.pack()

        self.steps_set = Tkinter.StringVar()
        self.steps_entry = Tkinter.Entry(self.calibra_panel, textvariable=self.steps_set)
        self.steps_entry.pack()

        self.step_left_btn = Tkinter.Button(self.calibra_panel, text="step left", command=self.step_left)
        self.step_left_btn.pack()

        self.step_right_btn = Tkinter.Button(self.calibra_panel, text="step right", command=self.step_right)
        self.step_right_btn.pack()

        self.std_len = Tkinter.StringVar()
        self.std_len_entry = Tkinter.Entry(self.calibra_panel, textvariable=self.std_len)
        self.std_len_entry.pack()

        self.change_std_len_btn = Tkinter.Button(self.calibra_panel, text="change std length", command=self.change_std_len)
        self.change_std_len_btn.pack()

        self.dist_in_std_len = Tkinter.StringVar()
        self.dist_in_std_len_entry = Tkinter.Entry(self.calibra_panel, textvariable=self.dist_in_std_len) 
        self.dist_in_std_len_entry.pack()

        self.go_dist_btn = Tkinter.Button(self.calibra_panel, text="go forward ", command=self.go_dist)
        self.go_dist_btn.pack()

        self.go_dist_btn = Tkinter.Button(self.calibra_panel, text="go backward", command=self.go_backward)
        self.go_dist_btn.pack()

        while True:
            self.calibra_panel.update()
            self.check_last_send()
            self.rcv_uart_msg()
            if self.state == "cycle":
                if time.time() - self.last_cycle_time >= 4:
                    self.cycle_state = self.cycle_pair[self.cycle_state]
                    self.Send_Direct_Order(order=self.cycle_state)
                    self.last_cycle_time = time.time()

if __name__ == '__main__':
    Admin = CarAdmin('Car')
    Admin.Run()

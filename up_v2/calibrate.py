#! /usr/bin/python2
# calibrate.py
'''
This software is for Tang's car.
'''
import os
import struct
import serial
import Tkinter
import time
import pickle
import pdb
import logging
from logging import handlers
from car_proxy import CarProxy
from const_var import const

log_name = 'Calibrate'

class App():


    def init_log(self):
        _LOG_FORMAT = '%(asctime)s (%(filename)s/%(funcName)s)' \
            ' %(name)s %(levelname)s - %(message)s'
        self.logger = logging.getLogger(log_name)
        _handler = logging.handlers.RotatingFileHandler(
            "./log/" +
            os.path.basename(__file__)[
                :-
                3] +
            ".log",
            maxBytes=102400,
            backupCount=20)
        _formatter = logging.Formatter(_LOG_FORMAT)
        _handler.setFormatter(_formatter)
        self.logger.addHandler(_handler)
        self.logger.setLevel(logging.INFO)
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        ch.setFormatter(_formatter)
        self.logger.addHandler(ch)

    def __init__(self):
        self.init_log()
        self.degree = 0

        self.init_panel()
        self.car = CarProxy(self.logger)

    def change_speed(self):
        speed = float(self.speed_entry.get())
        self.car.change_speed(speed)

    def forward(self):
        self.car.forward()

    def backward(self):
        self.car.backward()

    def forward_dist(self):
        dist = int(self.dist_entry.get())
        self.car.forward_dist(dist)

    def backward_dist(self):
        dist = int(self.dist_entry.get())
        self.car.backward_dist(dist)

    def forward_step(self):
        step = int(self.step_entry.get())
        self.car.forward_step(step)

    def backward_step(self):
        step = int(self.step_entry.get())
        self.car.backward_step(step)

    def stop(self):
        self.car.stop()

    def degree_left(self):
        degree = float(self.degree_entry.get())
        self.car.turn_left_degree(degree)

    def degree_right(self):
        degree = float(self.degree_entry.get())
        self.car.turn_right_degree(degree)

    def left_step(self):
        steps = int(self.step_entry.get())
        self.car.step_left(steps)

    def right_step(self):
        steps = int(self.step_entry.get())
        self.car.step_right(steps)

    def init_panel(self):
        self.map_panel = Tkinter.Tk()
        self.panel_now = self.map_panel

        self.Forward_Button = Tkinter.Button(self.panel_now, text="Forward",
                                             command=self.forward)
        self.Forward_Button.pack()

        self.Backward_button = Tkinter.Button(self.panel_now, text="Backward",
                                              command=self.backward)
        self.Backward_button.pack()

        self.Stop_button = Tkinter.Button(self.panel_now, text="stop",
                                          command=self.stop)
        self.Stop_button.pack()

        # degree turn 

        self.degree_set = Tkinter.StringVar()
        self.degree_entry = Tkinter.Entry(self.panel_now,
                                          textvariable=self.degree)
        self.degree_entry.pack()

        self.degree_left_btn = Tkinter.Button(self.panel_now,
                                            text="degree left",
                                            command=self.degree_left)
        self.degree_left_btn.pack()

        self.degree_right_btn = Tkinter.Button(self.panel_now, text="degree right",
                                             command=self.degree_right)
        self.degree_right_btn.pack()

        # speed 

        self.speed = Tkinter.DoubleVar()
        self.speed_entry = Tkinter.Entry(self.panel_now,
                                         textvariable=self.speed)
        self.speed_entry.pack()

        self.change_speed_btn = Tkinter.Button(self.panel_now,
                                               text="change speed",
                                             command=self.change_speed)
        self.change_speed_btn.pack()
        # go dist
        self.dist = Tkinter.StringVar()
        self.dist_entry = Tkinter.Entry(self.panel_now,
                                          textvariable=self.dist)
        self.dist_entry.pack()

        self.dist_forward_btn = Tkinter.Button(self.panel_now,
                                            text="forward dist",
                                            command=self.forward_dist)
        self.dist_forward_btn.pack()

        self.dist_back_btn = Tkinter.Button(self.panel_now,
                                             text="backward dist",
                                             command=self.backward_dist)
        self.dist_back_btn.pack()

        # go steps

        self.step = Tkinter.StringVar()
        self.step_entry = Tkinter.Entry(self.panel_now,
                                          textvariable=self.step)
        self.step_entry.pack()

        self.step_forward_btn = Tkinter.Button(self.panel_now,
                                            text="forward step",
                                            command=self.forward_step)
        self.step_forward_btn.pack()

        self.step_back_btn = Tkinter.Button(self.panel_now,
                                             text="backward step",
                                             command=self.backward_step)
        self.step_back_btn.pack()

        # turn steps
        self.step_left_btn = Tkinter.Button(self.panel_now,
                                            text="left step",
                                            command=self.left_step)
        self.step_left_btn.pack()

        self.step_right_btn = Tkinter.Button(self.panel_now,
                                             text="right step",
                                             command=self.right_step)
        self.step_right_btn.pack()


    def run(self):
        while True:
            self.panel_now.update()
            self.car.routines()


if __name__ == '__main__':
    app = App()
    app.run()

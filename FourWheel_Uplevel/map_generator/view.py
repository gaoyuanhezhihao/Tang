# view.py
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


class MapView():

    def __init__(self, logger, cmder):
        self.logger = logger
        self.degree = 0
        self.start_name = ''
        self.end_name = ''
        self.cmder = cmder
        self.init_map_panel()

    def forward(self):
        self.cmder.go_forward()

    def backward(self):
        self.cmder.go_backward()

    def stop(self):
        self.cmder.stop()

    def step_left(self):
        self.degree = int(self.degree_entry.get())
        self.cmder.step_left(self.degree)

    def step_right(self):
        self.degree = int(self.degree_entry.get())
        self.cmder.step_right(self.degree)

    def save_path(self):
        self.cmder.save_path()
        self.save_path_btn.config(state=Tkinter.DISABLED)
        self.start_new_route_btn.config(state=Tkinter.NORMAL)
        self.change_to_uninit_view()

    def change_to_move_view(self):
        self.start_new_route_btn.config(state=Tkinter.DISABLED)
        self.save_path_btn.config(state=Tkinter.NORMAL)
        self.Forward_Button.config(state=Tkinter.NORMAL)
        self.Backward_button.config(state=Tkinter.NORMAL)
        self.Stop_button.config(state=Tkinter.NORMAL)
        self.step_left_btn.config(state=Tkinter.NORMAL)
        self.step_right_btn.config(state=Tkinter.NORMAL)

    def change_to_uninit_view(self):
        self.start_new_route_btn.config(state=Tkinter.NORMAL)
        self.save_path_btn.config(state=Tkinter.DISABLED)
        self.Forward_Button.config(state=Tkinter.DISABLED)
        self.Backward_button.config(state=Tkinter.DISABLED)
        self.Stop_button.config(state=Tkinter.DISABLED)
        self.step_left_btn.config(state=Tkinter.DISABLED)
        self.step_right_btn.config(state=Tkinter.DISABLED)

    def new_route(self):
        self.start_name = str(self.start_name_entry.get())
        self.end_name = str(self.end_entry.get())
        self.cmder.new_route(self.start_name, self.end_name)
        self.change_to_move_view()

    def init_map_panel(self):
        self.map_panel = Tkinter.Tk()
        self.panel_now = self.map_panel

        self.Forward_Button = Tkinter.Button(self.panel_now, text="Forward",
                                             command=self.forward,
                                             state=Tkinter.DISABLED)
        self.Forward_Button.pack()

        self.Backward_button = Tkinter.Button(self.panel_now, text="Backward",
                                              command=self.backward,
                                              state=Tkinter.DISABLED)
        self.Backward_button.pack()

        self.Stop_button = Tkinter.Button(self.panel_now, text="stop",
                                          command=self.stop,
                                          state=Tkinter.DISABLED)
        self.Stop_button.pack()

        self.degree_set = Tkinter.StringVar()
        self.degree_entry = Tkinter.Entry(self.panel_now,
                                          textvariable=self.degree)
        self.degree_entry.pack()

        self.step_left_btn = Tkinter.Button(self.panel_now, text="step left",
                                            command=self.step_left,
                                            state=Tkinter.DISABLED)
        self.step_left_btn.pack()

        self.step_right_btn = Tkinter.Button(self.panel_now, text="step right",
                                             command=self.step_right,
                                             state=Tkinter.DISABLED)
        self.step_right_btn.pack()

        self.save_path_btn = Tkinter.Button(self.panel_now, text="save path",
                                            command=self.save_path,
                                            state=Tkinter.DISABLED)
        self.save_path_btn.pack()

        start_sign = Tkinter.StringVar()
        start_label = Tkinter.Label(self.panel_now, textvariable=start_sign,
                                    relief=Tkinter.RAISED)
        start_sign.set("start name")
        start_label.pack()

        self.start_name_set = Tkinter.StringVar()
        self.start_name_entry = Tkinter.Entry(self.panel_now,
                                              textvariable=self.start_name)
        self.start_name_entry.pack()

        end_sign = Tkinter.StringVar()
        end_label = Tkinter.Label(self.panel_now, textvariable=end_sign,
                                  relief=Tkinter.RAISED)
        end_sign.set("end name")
        end_label.pack()

        self.end_set = Tkinter.StringVar()
        self.end_entry = Tkinter.Entry(self.panel_now,
                                       textvariable=self.end_name)
        self.end_entry.pack()

        self.start_new_route_btn = Tkinter.Button(
            self.panel_now, text="start new route", command=self.new_route)
        self.start_new_route_btn.pack()

    def update_panel(self):
        self.panel_now.update()

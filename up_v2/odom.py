#!/usr/bin/python
from time import time
from math import cos, sin



class Odometry(object):
    _logger
    _prev_node = {}
    _x = 0.0
    _y = 0.0
    _theta = 90.0 #degree
    _vec_x, _vec_y = 0.0, 0.0
    _state = 's'
    updated = False

    def __vec_update():
        _vec_x, _vec_y = cos(self._theta), sin(self._theta)

    def __left(self, cm, step):
        dth = step / const.pulses_per_degree
        theta = self._theta
        theta += dth
        theta = theta - 360.0 if theta > 360.0 else theta
        self._theta = theta

    def __right(self, cm, step):
        dth = step / const.pulses_per_degree
        theta = self._theta
        theta -= dth
        theta = theta + 360.0 if theta < 0.0 else theta
        self._theta = theta

    def __forward(self, cm, step):
        self._x = _prev_node['x'] + cm * _vec_x
        self._y = _prev_node['y'] + cm * _vec_y

    def __back(self, cm, step):
        self._x = _prev_node['x'] - cm * _vec_x
        self._y = _prev_node['y'] - cm * _vec_y

    def __init__(self, logger):
        self._logger = logger
        self.dispath_map = {'l': self.__left, 'L': self.__left,
                            'r': self.__right, 'R':self.__right,
                            'f': self.__forward, 'F': self.__forward,
                            'b': self.__back, 'B': self.__back}

    def update(self, cm, step):
        if state in ('l', 'L', 'r', 'R', 'f', 'F', 'b', 'B'):
            self.dispath_map[state](cm, step)
            self.updated = True

    def state_change(self, new_state):
        self._state = new_state

    def get_odom(self):
        updated = False


#!/usr/bin/python
from time import time
from math import cos, sin, pi
import rospy


class Odometry(object):
    _logger = None
    _x = 0.0
    _y = 0.0
    _theta = pi/2  # 90 degree
    _vec_x, _vec_y = 0.0, 0.0
    _state = ''
    _time = 0.0
    updated = False

    def __vec_update(self):
        self._vec_x, self._vec_y = cos(self._theta), sin(self._theta)

    def __left(self, cm, step):
        dth = step / const.pulses_per_degree
        theta = self._theta
        theta = self._prev_theta + dth
        theta = theta - 360.0 if theta > 360.0 else theta
        self._theta = theta

    def __right(self, cm, step):
        dth = step / const.pulses_per_degree
        theta = self._theta
        theta = self._prev_theta - dth
        theta = theta + 360.0 if theta < 0.0 else theta
        self._theta = theta

    def __forward(self, cm, step):
        self._x = self._prev_x + cm * self._vec_x
        self._y = self._prev_y + cm * self._vec_y

    def __back(self, cm, step):
        self._x = self._prev_x - cm * self._vec_x
        self._y = self._prev_y - cm * self._vec_y

    def __init__(self, logger):
        self._logger = logger
        self.dispath_map = {'l': self.__left, 'L': self.__left,
                            'r': self.__right, 'R': self.__right,
                            'f': self.__forward, 'F': self.__forward,
                            'b': self.__back, 'B': self.__back}

    def update(self, cm, step):
        self._logger.info("cm=%d, step=%d" % (cm, step))
        if self._state in ('l', 'L', 'r', 'R', 'f', 'F', 'b', 'B'):
            self.dispath_map[self._state](cm, step)
            self._time = rospy.Time.now()
            self.updated = True

    def __save_old(self):
        self._prev_x = self._x
        self._prev_y = self._y
        self._prev_theta = self._theta
        self._prev_vec_x = self._vec_x
        self._prev_vec_y = self._vec_y

    def state_change(self, new_state):
        assert self._state != new_state
        self._state = new_state
        self.__vec_update()
        self.__save_old()

    def get_odom(self):
        self.updated = False
        self._logger.info("x=%d, y=%d" % (self._x, self._y))
        return self._x, self._y, self._theta, self._time

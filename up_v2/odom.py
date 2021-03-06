#!/usr/bin/python
from time import time
from math import cos, sin, pi
from config import Odom_Freq, Odom_steps_of_cm, steps_per_cm, steps_per_degree

_pls_radian = steps_per_degree * 180.0 / pi
frame_time  = 1.0/Odom_Freq
# import rospy

_2PI = 2*pi


def _readable_theta(theta):
    return theta * 180.0 / pi


class Odometry(object):
    _logger = None
    _x = 0.0
    _y = 0.0
    _theta = pi/2  # 90 degree
    _vec_x, _vec_y = 0.0, 0.0
    _state = ''
    _time = 0.0
    updated = False
    _vx = 0.
    _vy = 0.
    _vth = 0.

    def get_speed(self):
        return (self._vx, self._vy, self._vth) if 's' != self._state else (0, 0, 0)

    def __vec_update(self):
        self._vec_x, self._vec_y = cos(self._theta), sin(self._theta)

    def __left(self, cm, step):
        dth = step / _pls_radian
        self._theta = (self._prev_theta + dth)% (2*pi)
        self._vth = (self._theta - self._prev_odom_pack[2])/frame_time
        self._vx = self_vy = 0

    def __right(self, cm, step):
        dth = step / _pls_radian
        self._theta = (self._prev_theta - dth)%(2*pi)
        self._vth = (self._theta - self._prev_odom_pack[2])/frame_time
        self._vx = self_vy = 0

    def __forward(self, cm, step):
        self._x = self._prev_x + cm * self._vec_x
        self._y = self._prev_y + cm * self._vec_y
        self._vx = (self._x - self._prev_odom_pack[0])/frame_time
        self._vy = (self._y - self._prev_odom_pack[1])/frame_time
        self._vth = 0.0

    def __back(self, cm, step):
        self._x = self._prev_x - cm * self._vec_x
        self._y = self._prev_y - cm * self._vec_y
        self._vx = (self._x - self._prev_odom_pack[0])/frame_time
        self._vy = (self._y - self._prev_odom_pack[1])/frame_time
        self._vth = 0.0

    def __init__(self, logger):
        self._logger = logger
        self.dispath_map = {'l': self.__left, 'L': self.__left,
                            'r': self.__right, 'R': self.__right,
                            'f': self.__forward, 'F': self.__forward,
                            'b': self.__back, 'B': self.__back}

    def update(self, cm, step):
        self._logger.debug("cm=%d, step=%d" % (cm, step))
        total_steps = cm * Odom_steps_of_cm + step
        if self._state in ('l', 'L', 'r', 'R', 'f', 'F', 'b', 'B'):
            self._prev_odom_pack = (self._x, self._y, self._theta)
            self.dispath_map[self._state](total_steps/steps_per_cm, step)
            # self._time = rospy.Time.now()
            self._time = time()
            self.updated = True

    def __save_old(self):
        self._prev_x = self._x
        self._prev_y = self._y
        self._prev_theta = self._theta
        self._prev_vec_x = self._vec_x
        self._prev_vec_y = self._vec_y

    def state_change(self, new_state):
        self._logger.info("change state to %s" % new_state)
        assert self._state != new_state
        self._state = new_state
        self.__vec_update()
        self.__save_old()

    def get_odom(self):
        self.updated = False
        self._logger.info("x=%d, y=%d, theta=%f" % (self._x, self._y, _readable_theta(self._theta)))
        self._logger.info("vec_x=%f, vec_y=%f" % (self._vec_x, self._vec_y))
        return self._x/1000, self._y/1000, self._theta, self._time

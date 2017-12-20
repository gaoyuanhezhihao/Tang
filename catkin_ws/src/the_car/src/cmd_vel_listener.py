#!/usr/bin/python
from __future__ import print_function
import rospy, math, re, thread, time
from geometry_msgs.msg import Twist

from car_proxy import CarProxy

real_car = CarProxy()

def twistCallback(data):
    linear = data.linear.x
    angular = data.angular.z
    move(linear, angular)

def move(linear, angular):
    if linear == 0 and angular == 0:
        rospy.loginfo("stop")
        real_car.stop()
    elif linear > 0 and angular == 0:
        rospy.loginfo("forward")
        real_car.forward()
    elif linear < 0 and angular == 0:
        rospy.loginfo("backward")
        real_car.backward()
    elif linear == 0 and angular > 0:
        rospy.loginfo("left")
        real_car.turn_left()
    elif linear == 0 and angular < 0:
        rospy.loginfo("right")
        real_car.turn_right()
    elif linear != 0 and angular != 0:
        rospy.logerr("arc move isn't supported yet!")

def cleanup():
    rospy.loginfo("clean up")

rospy.init_node('cmd_vel_listener', anonymous=False)
rospy.on_shutdown(cleanup)

rospy.Subscriber("cmd_vel", Twist, twistCallback)
rospy.spin()

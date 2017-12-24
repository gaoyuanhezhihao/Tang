#!/usr/bin/python
from __future__ import print_function
import rospy, math, re, thread, time
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

from car_proxy import CarProxy

real_car = CarProxy()

speed = -10000.0
turn_speed = -10000.0


def twistCallback(data):
    linear = data.linear.x
    angular = data.angular.z
    move(linear, angular)


def move(linear, angular):
    global speed, turn_speed
    if linear == 0 and angular == 0:
        rospy.loginfo("stop")
        real_car.stop()
    elif linear > 0 and angular == 0:
        rospy.loginfo("forward")
        if linear != speed:
            speed = linear
            real_car.change_speed(speed)
        real_car.forward()
    elif linear < 0 and angular == 0:
        rospy.loginfo("backward")
        if abs(linear) != speed:
            speed = abs(linear)
            real_car.change_speed(speed)
        real_car.backward()
    elif linear == 0 and angular > 0:
        rospy.loginfo("left")
        if angular != turn_speed:
            turn = angular
            real_car.change_speed(turn)
        real_car.turn_left()
    elif linear == 0 and angular < 0:
        rospy.loginfo("right")
        if abs(angular) != turn_speed:
            turn = abs(angular)
            real_car.change_speed(turn)
        real_car.turn_right()
    elif linear != 0 and angular != 0:
        rospy.logerr("arc move isn't supported yet!")


def check_odom(odom_broadcaster, odom_pub):
    # rospy.loginfo("odm.updated="+str(real_car.odm.updated))
    if real_car.odm.updated:
        x, y, theta, time_stamp = real_car.odm.get_odom()
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        # tf transform
        odom_broadcaster.sendTransform((x, y, 0.), quat, time_stamp,
                                       "base_link", "odom")
        # odometry message
        odom = Odometry()
        odom.header.stamp = time_stamp
        odom.header.frame_id = "odom"
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*quat))

        vx, vy, vth = real_car.get_speed()
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
        odom_pub.publish(odom)


def cleanup():
    rospy.loginfo("clean up")


if __name__ == '__main__':
    rospy.init_node('cmd_vel_listener', anonymous=False)
    rospy.on_shutdown(cleanup)

    rospy.Subscriber("cmd_vel", Twist, twistCallback)
    # rospy.spin()
    r = rospy.Rate(1)
    odom_broadcaster = tf.TransformBroadcaster()
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    while not rospy.is_shutdown():
        real_car.routines()
        check_odom(odom_broadcaster, odom_pub)
        # rospy.loginfo("TODO: pub odometry")
        r.sleep()

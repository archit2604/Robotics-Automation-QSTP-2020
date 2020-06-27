#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

PI = 3.14159265359
rospy.init_node("controller")


def callback_path(msg):
    global x_g
    global y_g
    x_g = msg.x
    y_g = msg.y


def callback_odom(msg):
    twist = Twist()
    euler = euler_from_quaternion(
        quaternion=(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
    )
    global count
    global diff_sum
    global diff_prev
    global diff
    global e
    global e_prev
    global e_sum
    global x_g
    global y_g
    global diff
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    if count == 0:
        theta_g = math.atan2(y_g - y, x_g - x)
        diff = theta_g - euler[2]
        print(euler[2] * 180 / PI)
        diff_sum += diff
        dedt = diff - diff_prev
        twist.angular.z = 3 * (diff) + 0.00044 * (diff_sum) + 30 * dedt
        twist.linear.x = 0
        pub.publish(twist)
        diff_prev = diff
    if diff <= 0.0008 and diff >= -0.0008 and count == 0:
        twist.angular.z = 0
        pub.publish(twist)
        count = 1
        rospy.sleep(1)
    if count == 1:
        e = math.sqrt(((x_g - x) * (x_g - x)) + ((y_g - y) * (y_g - y)))
        print(e)
        e_sum += e
        dedt = e - e_prev
        twist.linear.x = (1.5 * (e) + 0.00044 * (e_sum) + 28 * dedt) / 30
        if twist.linear.x > 0.10:
            twist.linear.x = 0.10
        twist.angular.z = 0
        pub.publish(twist)
        e_prev = e
    if e <= 0.15 and e >= -0.15 and count == 1:
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)
        count = 2


x_g = 0
y_g = 0
sub_p = rospy.Subscriber("path", Point, callback_path)
global count
count = 0
global diff
diff = 1
global diff_sum
diff_sum = 0
global diff_prev
diff_prev = diff
global e
e = 1
global e_sum
e_sum = 0
global e_prev
e_prev = 0
sub_o = rospy.Subscriber("odom", Odometry, callback_odom)
pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
rospy.spin()

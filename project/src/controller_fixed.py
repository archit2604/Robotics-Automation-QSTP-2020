#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def callback(msg):
    pass


def callback_path(msg):
    global o
    global x_g
    global y_g
    global flag
    global l
    l = len(msg.poses)
    if l == 0:
        pass
    else:
        x_g = msg.poses[flag].pose.position.x
        y_g = msg.poses[flag].pose.position.y
    if o == 0:
        o = 1
        sub_odom = rospy.Subscriber("odom", Odometry, callback_odom)


def callback_odom(msg):
    global l
    global x_g
    global y_g
    global e_x_sum
    global e_x_prev
    global e_y_sum
    global e_y_prev
    global count1
    global count2
    global flag
    twist = Twist()
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    if count1 == 0:
        e_x = x_g - x
        e_x_sum += e_x
        dedt_x = e_x - e_x_prev
        u_x = 0.5 * e_x + 0.0008 * e_x_sum + 0.17 * dedt_x
        u_x *= 1.2
        if u_x > 0.7:
            u_x = 0.7
        elif u_x < -0.7:
            u_x = -0.7
        elif u_x < 0.05 and u_x > 0:
            u_x *= 3
        elif u_x > -0.05 and u_x < 0:
            u_x *= 3
        twist.linear.x = u_x
        pub.publish(twist)
        e_x_prev = e_x
        if e_x <= 0.05 and e_x >= -0.05:
            twist.linear.x = 0
            pub.publish(twist)
            count1 = 1
    if count2 == 0:
        e_y = y_g - y
        e_y_sum += e_y
        dedt_y = e_y - e_y_prev
        u_y = 0.5 * e_y + 0.0008 * e_y_sum + 0.17 * dedt_y
        u_y *= 1.2
        if u_y > 0.7:
            u_y = 0.7
        elif u_y < -0.7:
            u_y = -0.7
        elif u_y < 0.05 and u_y >= 0:
            u_y *= 3
        elif u_y > -0.05 and u_y <= 0:
            u_y *= 3
        twist.linear.y = u_y
        pub.publish(twist)
        e_y_prev = e_y
        if e_y <= 0.05 and e_y >= -0.05:
            twist.linear.y = 0
            pub.publish(twist)
            count2 = 1
    if count1 == 1 and count2 == 1:
        if flag + 1 < l:
            count1 = 0
            count2 = 0
            flag += 1
            rospy.sleep(0.1)


global count1
count1 = 0
global count2
count2 = 0
global e_x_sum
e_x_sum = 0
global e_x_prev
e_x_prev = 0
global e_y_sum
e_y_sum = 0
global e_y_prev
e_y_prev = 0
global flag
flag = 0
global o
o = 0
rospy.init_node("controller_ar")
sub_path = rospy.Subscriber("path", Path, callback_path)
pub = rospy.Publisher("cmd_vel", Twist, queue_size=2)
rospy.spin()

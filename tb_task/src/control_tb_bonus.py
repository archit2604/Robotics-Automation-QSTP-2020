#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from tb_task.srv import AngVel
from geometry_msgs.msg import Twist

PI = 3.14159265359


def callback(msg):
    rospy.wait_for_service("compute_ang_vel")
    proxy_ang_vel = rospy.ServiceProxy("compute_ang_vel", AngVel)
    ang = Float32()
    ang = proxy_ang_vel(msg.data)
    twist = Twist()
    while not rospy.is_shutdown():
        twist.linear.x = 0.2
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = ang.ang_vel
        pub.publish(twist)
        rospy.sleep(2 * PI / ang.ang_vel)
        twist.linear.x = 0.2
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -ang.ang_vel
        pub.publish(twist)
        rospy.sleep(2 * PI / ang.ang_vel)


rospy.init_node("control_tb")
pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)
sub = rospy.Subscriber("radius", Float32, callback)
rospy.spin()

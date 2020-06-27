#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from tb_task.srv import AngVel
from geometry_msgs.msg import Twist


def callback(msg):
    rospy.wait_for_service("compute_ang_vel")
    proxy_ang_vel = rospy.ServiceProxy("compute_ang_vel", AngVel)
    ang = Float32()
    ang = proxy_ang_vel(msg.data)
    twist = Twist()
    twist.linear.x = 0.2
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = ang.ang_vel
    pub.publish(twist)


rospy.init_node("control_tb")
pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)
sub = rospy.Subscriber("radius", Float32, callback)
rospy.spin()

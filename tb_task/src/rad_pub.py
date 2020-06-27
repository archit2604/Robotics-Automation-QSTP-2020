#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

rospy.init_node("radius_publisher")
pub = rospy.Publisher("radius", Float32, queue_size=0)
radius = float(input("Enter the radius:"))
rate = rospy.Rate(2)
while not rospy.is_shutdown():
    pub.publish(radius)
    rate.sleep()

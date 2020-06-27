#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point

rospy.init_node("path_planner")
pub = rospy.Publisher("path", Point, queue_size=1)
rate = rospy.Rate(10)
point = Point()
point.x = 0
point.y = 1
point.z = 0
while not rospy.is_shutdown():
    pub.publish(point)
    rate.sleep()

#!/usr/bin/env python
import rospy
from tb_task.srv import AngVel
def callback(request):
    ang_vel=0
    if request.radius!=0:
        ang_vel=1/request.radius
    return ang_vel
rospy.init_node('AngVelServer')
service=rospy.Service('compute_ang_vel',AngVel,callback)
rospy.spin()
#!/usr/bin/env python
import sys
import rospy
import std_msgs
import time
import math
import matplotlib.pyplot as plt
from gazebo_msgs.srv import *

def apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, \
        start_time, duration):
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    try:
        apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        apply_body_wrench(body_name, reference_frame, reference_point, wrench, \
        start_time, duration)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def clear_body_wrench_client(body_name):
    rospy.wait_for_service('gazebo/clear_body_wrenches')
    try:
        clear_body_wrench = rospy.ServiceProxy('gazebo/clear_body_wrenches', BodyRequest)
        clear_body_wrench(body_name)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    beginning = time.time()
    update_rate = 0.01
    period = 1
    body_name = 'billiyard_ball::link_0'
    reference_frame = 'world'
    start_time = rospy.Time(secs = 0, nsecs = 0)
    duration = rospy.Duration(secs = update_rate, nsecs = 0)
    reference_point = geometry_msgs.msg.Point(x = 0, y = 0, z = 0)
    t = []
    f = []
    # while time.time()-beginning <= 10:
    wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = -100, \
        # y = 10*math.cos(2*math.pi*(time.time()-beginning)/period), z = 0), \
        y = 0,z=-50),
        torque = geometry_msgs.msg.Vector3(x = 0, y = 0, z = 0))
    t.append(time.time() - beginning)
    f.append(wrench.force.y)
    clear_body_wrench_client(body_name)
    apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, \
        start_time, duration)
    time.sleep(update_rate - ((time.time()-beginning)%update_rate))

# plt.plot(t, f, drawstyle = "steps-post")
# plt.show()
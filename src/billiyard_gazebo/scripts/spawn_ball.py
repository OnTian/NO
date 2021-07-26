#!/usr/bin/env python
import sys
import tf.transformations
import rospy
import std_msgs
import time
import math
from gazebo_msgs.srv import *

def clear_model_client(model_name):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        clear_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        clear_model(model_name)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def spawn_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame):
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    try:
        spawn_model = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        spawn_model(model_name, model_xml, robot_namespace, initial_pose, reference_frame)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    beginning = time.time()
    update_rate = 0.01
    period = 1
    model_name = "billiyard_ball"
    x = 0
    y = 0
    z = 0.5
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    orient = geometry_msgs.msg.Quaternion(*q)
    with open("/home/onda/ball_ws/src/billiyard_gazebo/models/billiyard_ball/model.sdf", "r") as f:
        model_xml = f.read()
    robot_namespace = ""
    reference_frame = "world"
    clear_model_client(model_name)
    spawn_model_client(model_name, model_xml, robot_namespace, geometry_msgs.msg.Pose(geometry_msgs.msg.Point(0.4, 0, 0), orient), reference_frame)

    a = int(input('want to spawn model, enter 1'))
    if (a == 1):
        for i in range(15):
            model_name = "billiyard_ball" + str(i)
            clear_model_client(model_name)
        k = 0
        for i in range(5):
            for j in range(i+1):
                model_name = "billiyard_ball" + str(k)
                initial_pose = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(x, y + 0.05*j, 0), orient)
                spawn_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame)
                k = k + 1
            x = x - 0.05001*math.cos(30*math.pi/180)
            y = y - 0.05001*math.sin(30*math.pi/180)

    time.sleep(update_rate - ((time.time()-beginning)%update_rate))

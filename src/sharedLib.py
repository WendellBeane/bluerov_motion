#!/usr/bin/env python
import rospy

from mavros_msgs.srv import SetMode
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import ParamSet
from mavros_msgs.srv import CommandBool



# service clients
change_param = rospy.ServiceProxy('/mavros/param/set', ParamSet)
change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)


############ shared functions #############
def establish_ROS_control():
    myparam = ParamValue()

    # To have control from ROS
    rospy.wait_for_service('/mavros/param/set')
    try:
        myparam.integer = 1
        myparam.real = 0
        out = change_param("SYSID_MYGCS", myparam)
        if out.success:
            rospy.loginfo("ROS control esablished")
        else:
            rospy.loginfo("Failed gaining ROS control")
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed")


def establish_QGC_control():
    myparam = ParamValue()

    # To give control back to QGoundControl
    rospy.wait_for_service('/mavros/param/set')
    try:
        myparam.integer = 255
        myparam.real = 0
        out = change_param("SYSID_MYGCS", myparam)
        if out.success:
            rospy.loginfo("QGroundControl control established")
        else:
            rospy.loginfo("Failed gaining QGroundControl control")
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed")


def setMode(mode="MANUAL"):
    # change mode
    rospy.wait_for_service('/mavros/set_mode')
    try:
        base_mode = 0
        custom_mode = mode
        out = change_mode(base_mode, custom_mode)
        if out.mode_sent:
            rospy.loginfo(mode + " mode set")
        else:
            rospy.loginfo("Failed SetMode")
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed")
    last_request = rospy.get_rostime() 

    while not out.mode_sent:
        r.sleep()
        out = change_mode(base_mode, custom_mode)
        if out.mode_sent:
            rospy.loginfo("setmode send ok value")
        else:
            rospy.loginfo("Failed SetMode")

def armRobot():
    # ARM the robot
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        out = arm(True)
        if out.success:
            rospy.loginfo("Armed")
        else:
            rospy.loginfo("Failed Arming")
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed")
    last_request = rospy.get_rostime() 

    while not out.success:
        r.sleep()
        out = arm(True)
        if out.success:
            rospy.loginfo("Armed")
        else:
            rospy.loginfo("Failed Arming")

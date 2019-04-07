import rospy
import time
import numpy as np

from controller_functions import move
from ping_nodelet.msg import Ping

#safe distance in centemeters
SAFE_DISTANCE = 0.25

class Ping_Controller:

    def __init__(self):
        rospy.init_node("PingController")
        self.ping_sub = rospy.Subscriber('ping_nodelet/ping', Ping, self.ping_callback)
        self.safe = True

    def spin(self):
        while not self.safe:
            rospy.loginfo("not safe")
            move("backward", duration_seconds = 1)
            rospy.sleep(1)

    def ping_callback(self, msg):

        if msg.distance <= SAFE_DISTANCE:
            self.safe = False
            rospy.loginfo(msg.distance)
            rospy.loginfo("Too close to object will try to move away from it")
        else:
            self.safe = True


def main():
    robot = Ping_Controller()
    rate = rospy.Rate(10)

    #move("rotate left", duration_seconds = 3)
    while not rospy.is_shutdown():
        rate.sleep()

main()

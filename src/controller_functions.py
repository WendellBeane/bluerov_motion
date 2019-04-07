import rospy
import time
import numpy as np

from mavros_msgs.msg import OverrideRCIn
from constants import (neutral_command, CHANNEL_INFO, RC_NEUTRAL)
from sharedLib import establish_QGC_control, armRobot, setMode
PUBLISH_RATE=100

def _get_command(direction_to_speed):
    """ Converts mappings such as { right: 50 } into appropriate OverrideRCIn command.
            The input dictionary CAN have multiple key-value pairs.  """
    cmd = neutral_command()
    display_string = ""
    for direction, speed, in direction_to_speed.iteritems():
        channel, modifier = CHANNEL_INFO[direction]
        cmd.channels[channel] = RC_NEUTRAL + speed * modifier
        display_string += direction + ": " + str(speed) + "\t"
    print(display_string)
    return cmd

def move(direction, duration_seconds = 1, speed=50):
    """ Moves in a single direction, as specified by a string
        (e.g., "forward", or "right"). """
    movement = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
    establish_QGC_control()
    setMode("MANUAL")
    armRobot()

    cmd = _get_command({direction:speed})

    start = rospy.get_time()
    rate = rospy.Rate(PUBLISH_RATE)
    while not rospy.is_shutdown() and rospy.get_time() - start < duration_seconds:
        movement.publish(cmd)
        rate.sleep()

    movement.publish(neutral_command())

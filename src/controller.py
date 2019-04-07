#!/usr/bin/env python
import rospy
import time
import numpy as np
import json


from mavros_msgs.msg import OverrideRCIn
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

from constants import (neutral_command, CHANNEL_INFO, RC_NEUTRAL)
from sharedLib import establish_QGC_control, armRobot, setMode
from speed_generators import NORM, INDIVIDUAL, YAW_ONLY, XYZ_ONLY, SWITCHER
from helpers import normalize, array_to_string
from serialization import serialize

# We have experimentally determined that a rate of 100
# works well; it's one of the slowest rates that still
# produces consistent (uninterrupted) movement.
PUBLISH_RATE = 100


class Controller:

    def __init__(self):

        rospy.init_node("Controller")
        self.movement = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self._update_pose)

        self.poses = []         # previous states: (time, position, orientation).
                                # If no marker was found, the 3-tuple will be a single
                                # string "NO MARKER" instead.

        self.move_cmd = None
        self.goal_pose = None   # (position, orientation)
        self.speed_generator = SWITCHER
        self.direction_to_speed = None # TODO remove


    def move_to(self, pose, max_duration = 10):
        """ Here, pose is a tuple of numpy arrays: (position, orientation).
            max_duration is in seconds.  """

        establish_QGC_control()
        setMode("MANUAL")
        armRobot()

        self.goal_pose = pose

        start = rospy.get_time()
        rate = rospy.Rate(PUBLISH_RATE)
        while not rospy.is_shutdown() and rospy.get_time() - start < max_duration:
            if self.direction_to_speed == None or len(self.poses[-1]) != 3: # TODO clean this printing.
                continue
            display_string = ""
            for direction, speed, in self.direction_to_speed.iteritems():
                display_string += direction + ": {0:6.2f}".format(speed) + "\t"
            print(display_string)

            _, curr_xyz, curr_orient = self.poses[-1]
            goal_xyz, goal_orient = self.goal_pose
            delta_xyz = goal_xyz - curr_xyz
            delta_orient = goal_orient - curr_orient
            print("\t\t\t\t\t\t\t\t\t\t" + array_to_string(delta_xyz) + array_to_string(delta_orient))

            # TODO have we reached goal pose?
                # TODO break loop

            if self.move_cmd is not None:
                self.movement.publish(self.move_cmd)

            rate.sleep()

        if rospy.get_time() - start >= max_duration:
            print("Stopped because the maximum duration was exceeded; "
                + "may NOT have reached the position. ")

        self.goal_pose = None
        self.movement.publish(neutral_command())


    def move(self, direction, duration_seconds = 1, speed=50):
        """ Moves in a single direction, as specified by a string
            (e.g., "forward", or "right"). """

        establish_QGC_control()
        setMode("MANUAL")
        armRobot()

        cmd = self._get_command({direction:speed})

        start = rospy.get_time()
        rate = rospy.Rate(PUBLISH_RATE)
        while not rospy.is_shutdown() and rospy.get_time() - start < duration_seconds:

            self.movement.publish(cmd)
            rate.sleep()

        self.movement.publish(neutral_command())


    def _get_command(self, direction_to_speed):
        """ Converts mappings such as { right: 50 } into appropriate OverrideRCIn command.
            The input dictionary CAN have multiple key-value pairs.  """

        self.direction_to_speed = direction_to_speed # TODO Remove.
        cmd = neutral_command()
        display_string = ""
        for direction, speed, in direction_to_speed.iteritems():
            channel, modifier = CHANNEL_INFO[direction]
            cmd.channels[channel] = RC_NEUTRAL + speed * modifier
            display_string += direction + ": {0:6.2f}".format(speed) + "\t"
        #print(display_string)
        return cmd


    def _update_pose(self, msg):
        """ Called when the AR-tag channel publishes. """

        if len(msg.markers) > 0:

            time = msg.markers[0].header.stamp.secs
            pose = msg.markers[0].pose.pose
            position, orient = pose.position, pose.orientation

            # NOTE: we are SWAPPING y and z axes; it's more sensible
            # to have the Z axis, instead of the Y, be pointing up.
            position = np.array([position.x, position.z, position.y])

            quat_list = [orient.x, orient.y, orient.z, orient.w]
            (roll, pitch, yaw) = euler_from_quaternion(quat_list)

            # NOTE: we are SWAPPING y and z axes; it's more sensible
            # to have the Z axis, instead of the Y, be pointing up.
            orientation = np.array([roll, yaw, pitch])

            self.poses.append((time, position, orientation))

            if self.goal_pose is not None:
                self._update_move_cmd()

        else:
            print("No marker")
            self.poses.append("NO MARKER")


    def _update_move_cmd(self):
        _, curr_xyz, curr_orient = self.poses[-1]
        goal_xyz, goal_orient = self.goal_pose
        delta_xyz = goal_xyz - curr_xyz
        delta_orient = goal_orient - curr_orient

        #print("\t\t\t\t\t\t\t\t\t\t" + array_to_string(delta_xyz) + array_to_string(delta_orient))

        speeds = self.speed_generator((delta_xyz, delta_orient))
        ((x, y, z), (roll, pitch, yaw)) = speeds

        self.move_cmd = self._get_command({
            "rotate left": yaw,
            "up": z,
            "backward": y,
            "left": x})
        # NOTE: we are IGNORING roll and pitch for now


    def serialize_poses(self):
        serialize(self.poses)


    # TODO: move this function into another file
    # (since we'll now be keeping locations in a field of the controller obj)
    def get_marker_pos(self):

        try:
            msg = rospy.wait_for_message('ar_pose_marker', AlvarMarkers, timeout=5)
            if len(msg.markers) == 0:
                print("Error getting marker position -- no markers visible.")
            elif len(msg.markers) > 1:
                print("Error getting marker position -- more than 1 marker visible")
            else:
                pose = msg.markers[0].pose.pose
                position, orient = pose.position, pose.orientation

                pos_array = np.array([position.x, position.y, position.z])
                quat_list = [orient.x, orient.y, orient.z, orient.w]
                (roll, pitch, yaw) = euler_from_quaternion(quat_list)
                euler = np.array([roll, pitch, yaw])

                return (pos_array, euler)

        except rospy.ROSException:
            print("Error getting marker position -- no message after several seconds. " +
                  "Make sure that the topic exists and is publishing.")

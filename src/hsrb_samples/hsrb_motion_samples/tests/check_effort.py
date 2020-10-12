#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation
import sys
import unittest

import roslib
import rospy
from sensor_msgs.msg import JointState

roslib.load_manifest('rostest')


class PoseTest(unittest.TestCase):
    target = {}

    def __init__(self, *args):
        super(PoseTest, self).__init__(*args)
        rospy.loginfo("test initialized")

    def jointStatesInput(self, input):
        values = {}
        for i in range(0, len(input.name)):
            values[input.name[i]] = input.effort[i]
        pos_error = 0.0
        rospy.loginfo("target num: %i", len(self.target))
        for (j, v) in self.target.items():
            rospy.loginfo("joint: %s, current: %f, target: %f",
                          j, values[j], v)
            pos_error = pos_error + abs(values[j] - v)

        rospy.loginfo("error effort: %f", pos_error)
        if pos_error < 0.01:
            rospy.loginfo("success")
            self.success = True

    def test_pose(self):
        rospy.loginfo("subscribe to topic")
        self.success = False
        rospy.Subscriber("/hsrb/joint_states", JointState,
                         self.jointStatesInput)
        while not rospy.is_shutdown() and self.success is False:
            rospy.sleep(0.1)
        self.assertTrue(self.success)

if __name__ == '__main__':
    import rostest
    rospy.init_node("check_pose", anonymous=True)
    for a in sys.argv[1:]:
        (j, v) = a.split(":")
        rospy.loginfo("target joint: %s, value: %s", j, v)
        try:
            PoseTest.target[j] = float(v)
        except ValueError:
            rospy.loginfo("fail to set target joint: %s, value: %s", j, v)
    rostest.run("hsrb_motion_samples", sys.argv[0], PoseTest, sys.argv)

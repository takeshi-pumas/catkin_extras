#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation
import sys
import unittest

from nav_msgs.msg import Odometry
import roslib
import rospy

roslib.load_manifest('rostest')


class OdomTest(unittest.TestCase):
    target = {}

    def __init__(self, *args):
        super(OdomTest, self).__init__(*args)
        rospy.loginfo("test initialized")

    def odometryInput(self, input):
        values = {}
        values['x'] = input.pose.pose.position.x
        values['y'] = input.pose.pose.position.y
        values['z'] = input.pose.pose.position.z
        pos_error = 0.0
        rospy.loginfo("target num: %i", len(self.target))
        for (j, v) in self.target.items():
            rospy.loginfo("axis: %s, current: %f, target: %f", j, values[j], v)
            pos_error = pos_error + abs(values[j] - v)

        rospy.loginfo("error pos: %f", pos_error)
        if pos_error < 0.1:
            rospy.loginfo("success")
            self.success = True

    def test_odom(self):
        rospy.loginfo("subscribe to topic")
        self.success = False
        rospy.Subscriber("/hsrb/odom", Odometry, self.odometryInput)
        while not rospy.is_shutdown() and self.success is False:
            rospy.sleep(0.1)
        self.assertTrue(self.success)

if __name__ == '__main__':
    import rostest
    rospy.init_node("check_odom", anonymous=True)
    for a in sys.argv[1:]:
        (j, v) = a.split(":")
        rospy.loginfo("target axis: %s, value: %s", j, v)
        try:
            OdomTest.target[j] = float(v)
        except ValueError:
            rospy.loginfo("fail to set target axis: %s, value: %s", j, v)
    rostest.run("hsrb_motion_samples", sys.argv[0], OdomTest, sys.argv)

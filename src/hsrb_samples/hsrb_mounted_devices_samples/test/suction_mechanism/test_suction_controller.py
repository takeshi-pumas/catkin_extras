#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2017 Toyota Motor Corporation
import unittest

import hsrb_mounted_devices_samples.suction_mechanism.suction_controller\
    as node
from rosgraph_msgs.msg import Log
import rospy
import rostest
from tmc_suction.msg import SuctionControlGoal


class TestSuctionController(unittest.TestCase):

    def setUp(self):
        # Create goal subscriber to get suction server's input
        self._goal_list = []
        self._goal_sub = rospy.Subscriber("/goal_msg_for_test",
                                          SuctionControlGoal,
                                          self._server_goal_cb)

        # Create rosout subscriber to get node's output
        self._log_message_list = []
        self._rosout_sub = rospy.Subscriber("/rosout", Log, self._rosout_cb)
        while self._rosout_sub.get_num_connections() < 2:
            rospy.sleep(0.1)

        # Because rosout is a latched topic, delete saved message
        rospy.sleep(1.0)
        self._log_message_list = []

    def tearDown(self):
        self._goal_sub.unregister()
        self._rosout_sub.unregister()

    def _server_goal_cb(self, action_goal):
        self._goal_list.append(action_goal)

    def _rosout_cb(self, log):
        if log.name == '/test_suction_controller':
            self._log_message_list.append(log.msg)

    def test_suction_succeeded(self):
        rospy.set_param('result_status', 'SUCCEEDED')
        node.main()
        # Wait for spin rosout callback
        rospy.sleep(1.0)

        self.assertEqual(len(self._goal_list), 2)
        # self._goal_list[0]: suction_on request
        self.assertEqual(self._goal_list[0].timeout.secs, 20)
        self.assertTrue(self._goal_list[0].suction_on.data)
        # self._goal_list[1]: suction_off request
        self.assertEqual(self._goal_list[1].timeout.secs, 0)
        self.assertFalse(self._goal_list[1].suction_on.data)

        self.assertEqual(len(self._log_message_list), 2)
        self.assertEqual(self._log_message_list[0], 'Suction will start')
        self.assertEqual(self._log_message_list[1],
                         'Suction succeeded. Suction will stop')

    def test_suction_failed(self):
        rospy.set_param('result_status', 'ABORTED')
        node.main()
        # Wait for spin rosout callback
        rospy.sleep(1.0)

        self.assertEqual(len(self._goal_list), 1)
        # self._goal_list[0]: suction_on request
        self.assertEqual(self._goal_list[0].timeout.secs, 20)
        self.assertTrue(self._goal_list[0].suction_on.data)

        self.assertEqual(len(self._log_message_list), 2)
        self.assertEqual(self._log_message_list[0], 'Suction will start')
        self.assertEqual(self._log_message_list[1], 'Suction failed')

if __name__ == '__main__':
    rospy.init_node('test_suction_controller')
    rostest.rosrun('hsrb_mounted_devices_samples',
                   'suction_controller_test',
                   TestSuctionController)

#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2017 Toyota Motor Corporation
import sys

import actionlib
import rospy
from tmc_suction.msg import (
    SuctionControlAction,
    SuctionControlGoal
)


class SuctionActionStub(object):
    def __init__(self):
        # Create goal publisher for send goal data to test node
        self._goal_pub = rospy.Publisher('/goal_msg_for_test',
                                         SuctionControlGoal,
                                         queue_size=10)

        # Wait until test node will create goal subscriber
        while self._goal_pub.get_num_connections() == 0:
            rospy.sleep(0.1)

        self._as = actionlib.SimpleActionServer("/hsrb/suction_control",
                                                SuctionControlAction,
                                                execute_cb=self.execute_cb,
                                                auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        self._goal_pub.publish(goal)

        status = rospy.get_param('result_status', 'UNDEFINED')
        if status == 'SUCCEEDED':
            self._as.set_succeeded()
        elif status == 'ABORTED':
            self._as.set_aborted()
        else:
            rospy.logerr('Parameter is not set')
            sys.exit(1)

if __name__ == '__main__':
    rospy.init_node('suction_action_server_stub')
    SuctionActionStub()
    rospy.spin()

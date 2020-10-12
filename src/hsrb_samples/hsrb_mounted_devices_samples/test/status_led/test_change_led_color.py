#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2017 Toyota Motor Corporation
import unittest

import rospy
import rostest
from std_msgs.msg import ColorRGBA

_LED_INPUT_DATA_SIZE = 256


class TestChangeLEDColor(unittest.TestCase):
    def setUp(self):
        # Hold subscribed time and rgb value of status led topic
        self._time_list = []
        self._led_value_list = []

        # Create subscriber for status led topic
        self._led_sub = rospy.Subscriber('/hsrb/command_status_led_rgb',
                                         ColorRGBA,
                                         self._led_callback,
                                         queue_size=100)

        # Wait for connection
        while self._led_sub.get_num_connections() == 0:
            rospy.sleep(0.1)

    def tearDown(self):
        self._led_sub.unregister()

    def _led_callback(self, rgb):
        self._time_list.append(rospy.Time.now())
        self._led_value_list.append(rgb)

    def test_led_msg_ok(self):
        # Wait until test node is finished
        while self._led_sub.get_num_connections() > 0:
            rospy.sleep(0.1)

        # R value check
        # r = x / (_LED_INPUT_DATA_SIZE - 1)
        # {x | 0 <= x <= _LED_INPUT_DATA_SIZE - 1}
        self.assertEqual(len(self._led_value_list), _LED_INPUT_DATA_SIZE)
        for x, led_value in enumerate(self._led_value_list):
            expect_value = x / float(_LED_INPUT_DATA_SIZE - 1)
            self.assertAlmostEqual(led_value.r, expect_value)

        # Frequency check
        duration = (self._time_list[-1] - self._time_list[0]).to_sec()
        frequency = len(self._led_value_list) / duration
        self.assertAlmostEqual(frequency, 50, places=0)


if __name__ == '__main__':
    rospy.init_node('change_led_color_test')
    rostest.rosrun('hsrb_mounted_devices_samples',
                   'change_led_color_test',
                   TestChangeLEDColor)

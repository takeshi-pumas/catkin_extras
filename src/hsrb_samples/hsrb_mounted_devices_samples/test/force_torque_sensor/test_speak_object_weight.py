#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2016 Toyota Motor Corporation
import os
import unittest

import hsrb_mounted_devices_samples\
    .force_torque_sensor.speak_object_weight as node
from mock import patch
import rospy
import rostest


_PKG = 'hsrb_mounted_devices_samples.force_torque_sensor.speak_object_weight'


@patch('actionlib.SimpleActionClient')
@patch('rospy.wait_for_message')
@patch('rospy.ServiceProxy')
@patch(_PKG + '.ForceSensorCapture')
@patch(_PKG + '.Speaker.speak_sentence')
class TestSpeakObjectWeight(unittest.TestCase):
    def test_main_japanese(self,
                           speak_sentence_mock,
                           sensor_capture_mock,
                           service_client_mock,
                           wait_for_msg_mock,
                           action_client_mock):
        # define mock behavior
        wait_for_msg_mock.return_value = True
        service_client_mock.return_value.wait_for_service.side_effect = "ok"
        action_client_mock.return_value.wait_for_server.side_effect = "ok"

        # set language
        os.environ['LANG'] = 'ja_JP.UTF-8'

        # define pre force data and post force data
        sensor_capture_mock.return_value.get_current_force.side_effect = [
            [0.0, 0.0, 0.0], [5.0, 0.1, 0.1]]
        # sqrt((5.0-0.0)^2+(0.1-0.0)^2+(0.1-0.0)^2))/9.81*1000
        expect_output = u'これは509.9グラムです'

        # call main function
        node.main()

        speak_sentence_mock.assert_called_with(expect_output)

    def test_main_english(self,
                          speak_sentence_mock,
                          sensor_capture_mock,
                          service_client_mock,
                          wait_for_msg_mock,
                          action_client_mock):
        # define mock behavior
        wait_for_msg_mock.return_value = True
        service_client_mock.return_value.wait_for_service.side_effect = "ok"
        action_client_mock.return_value.wait_for_server.side_effect = "ok"

        # set language
        os.environ['LANG'] = 'en_US.UTF-8'

        # define pre force data and post force data
        sensor_capture_mock.return_value.get_current_force.side_effect = [
            [0.0, 0.0, 0.0], [5.0, 0.1, 0.1]]
        # sqrt((5.0-0.0)^2+(0.1-0.0)^2+(0.1-0.0)^2))/9.81*1000
        expect_output = 'This is 509.9 gram'

        # call main function
        node.main()

        # check calculated object_weight
        speak_sentence_mock.assert_called_with(expect_output)

if __name__ == '__main__':
    rospy.init_node('speak_object_weight_test')
    rostest.rosrun('hsrb_mounted_devices_samples',
                   'speak_object_weight_test',
                   TestSpeakObjectWeight)

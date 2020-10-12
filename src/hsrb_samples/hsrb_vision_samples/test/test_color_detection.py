#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation
import unittest

from hsrb_vision_samples.color_detection import ColorDetection
from nose.tools import eq_
from nose.tools import raises
import rospy
import rostest
from sensor_msgs.msg import Image


class TestColorDetection(unittest.TestCase):
    def setUp(self):
        self._img_pub = rospy.Publisher('/hsrb/head_rgbd_sensor'
                                        '/rgb/image_rect_color',
                                        Image, latch=True,
                                        queue_size=1)

    @raises(rospy.ROSException)
    def test_cannot_subscribe(self):
        ColorDetection()

    def test_ok(self):
        """create sample picture for test"""
        sample_img = Image()
        sample_img.header.stamp = rospy.Time.now()
        sample_img.height = 3
        sample_img.width = 4
        sample_img.encoding = 'bgr8'
        sample_img.is_bigendian = 0
        sample_img.step = 12
        sample_img.data = [0] * (sample_img.step * sample_img.height)

        self._img_pub.publish(sample_img)
        instance = ColorDetection()
        dst_img = instance.extract_color()
        # dst_img type is numpy array
        eq_(dst_img.shape[0] * dst_img.shape[1],
            sample_img.width * sample_img.height)


if __name__ == '__main__':
    rospy.init_node('color_detection_sample')
    rostest.rosrun('hsrb_vision_samples',
                   'color_detection_sample',
                   TestColorDetection)

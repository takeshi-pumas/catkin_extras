Overview
++++++++

Provided Functions
-------------------

- Skintone detection using HSV coordinates of the color image

- Plane detection using point cloud data

ROS Interface
++++++++++++++

Nodes
-----

- **hsrb_color_detection** Skintone detection node

- **hsrb_plane_detection** Plane detection node


Subscribed Topics
^^^^^^^^^^^^^^^^^

- **/hsrb/head_rgbd_sensor/rgb/image_rect_color** (:ros:msg:`sensor_msgs.msg/Image`) Color image

- **/hsrb/head_rgbd_sensor/depth_registered/rectified_points** (:ros:msg:`sensor_msgs.msg/PointCloud2`) Point cloud data


Published Topics
^^^^^^^^^^^^^^^^^

- **/plane_detection_output** (:ros:msg:`sensor_msgs.msg/PointCloud2`)  Point cloud data where the detected plane was colored red


How to use
++++++++++

Use the skintone detection program
-----------------------------------

Python

.. code-block:: bash

   rosrun hsrb_vision_samples color_detection.py

C++

.. code-block:: bash

   rosrun hsrb_vision_samples hsrb_color_detection

Use the plane detection program
---------------------------------

C++

.. code-block:: bash

   rosrun hsrb_vision_samples hsrb_plane_detection


Internal
++++++++

.. ifconfig:: internal

   Behavior:
   * Detect the skintone regions in realtime from a subscribed color image, and display the detected regions in white and the undetected regions in black.

   * Detect the plane in realtime from the subscribed point cloud data, and color the corresponding points red.

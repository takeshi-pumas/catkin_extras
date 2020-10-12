Overview
++++++++

Samples using HSRB on-board devices


Provided Functions
------------------

- Use the suction mechanism

- Measure the weight of a grasped object from the value of the force sensor and announce the result

- Light the status indicator LED gradually in steps

ROS Interface
++++++++++++++

Nodes
-----

- **hsrb_suction_controller** Node that calls the suction_control action

Action Client
^^^^^^^^^^^^^

- **/hsrb/suction_control** (:ros:msg:`tmc_suction/SuctionControlAction`)

  Control suction mechanism for HSRB

Nodes
-----

- **hsrb_speak_object_weight** Node that says the weight of a grasped object

Action Client
^^^^^^^^^^^^^

- **/hsrb/gripper_controller/grasp** (:ros:msg:`tmc_control_msgs/GripperApplyEffortAction`)

  HSRB gripper torque control action client

- **/talk_request_action** (:ros:msg:`tmc_msgs/TalkRequestAction`)

  Action client for text to speech

Service Client
^^^^^^^^^^^^^^

- **/safe_pose_changer/change_joint** (:ros:srv:`tmc_manipulation_msgs/SafeJointChange`)

  Service client for changing joint position

Subscribed Topics
^^^^^^^^^^^^^^^^^

- **/hsrb/wrist_wrench/raw** (:ros:msg:`geometry_msgs/WrenchStamped`)

  Subsclibe force torque sensor's raw data

Nodes
-----

- **hsrb_change_led_color** Node for lighting status indicator LED gradually in steps

Published Topics
^^^^^^^^^^^^^^^^^

- **/hsrb/command_status_led_rgb** (:ros:msg:`std_msgs/ColorRGBA`)

  Publish RGB data to status indicator LED


How to use
++++++++++

Calling the suction action
--------------------------

Python

.. code-block:: bash

   rosrun hsrb_mounted_devices_samples suction_controller.py

C++

.. code-block:: bash

   rosrun hsrb_mounted_devices_samples hsrb_suction_controller

Speak the weight of the grasped object
---------------------------------------

Python

.. code-block:: bash

   rosrun hsrb_mounted_devices_samples speak_object_weight.py

C++

.. code-block:: bash

   rosrun hsrb_mounted_devices_samples hsrb_speak_object_weight

Status indicator LED color changes
-----------------------------------

Python

.. code-block:: bash

   rosrun hsrb_mounted_devices_samples change_led_color.py

C++

.. code-block:: bash

   rosrun hsrb_mounted_devices_samples hsrb_change_led_color


Internal
++++++++

Calling the suction action
--------------------------

.. ifconfig:: internal

   Behavior:
   * Send a request to suction_control action
   * If the suction_control action returns true, stop suction and if it returns false, exit with a timeout error.

Speak the weight of the grasped object
---------------------------------------

.. ifconfig:: internal

   Behavior:
   * Transition to a pose which can grasp an object
   * Close the gripper (when it is sure that there is an object inside the gripper)
   * The weight of the grasped object is calculated from the difference between the measured values of the force sensor before and after the object is grasped
   * The HSR says the result

Status indicator LED color changes
----------------------------------
.. ifconfig:: internal

   Behavior:
   * Lighting status indicator LED gradationally

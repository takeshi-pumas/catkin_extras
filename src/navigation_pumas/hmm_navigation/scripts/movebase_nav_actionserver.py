#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: oscar
"""
"""
Modified HMM navigation server to follow a sequence of coordinates using MoveBaseAction.
"""
import rospy
import actionlib
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Quaternion, Point
import tf
from hmm_nav_utils import *  # Assuming you need additional utilities from this file

class HMMNavMoveBaseServer:
    def __init__(self):
        self.hmm_nav_server = actionlib.SimpleActionServer(
            "navigate_hmm", NavigateAction, execute_cb=self.execute_cb, auto_start=False
        )
        self.nav_client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        self.listener = tf.TransformListener()
        self.hmm_nav_server.start()

        # Load centroids (path points) from the file
        self.centroids = ccxyth
        rospy.loginfo("Centroids loaded for navigation.")

    def send_move_base_goal(self, x, y, yaw, timeout=20):
        """
        Send a single MoveBaseAction goal to the move_base server.
        """
        pose = PoseStamped()
        pose.header.stamp = rospy.Time(0)
        pose.header.frame_id = "map"
        pose.pose.position = Point(x, y, 0)
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation = Quaternion(*quat)

        goal = MoveBaseGoal()
        goal.target_pose = pose

        rospy.loginfo(f"Sending MoveBase goal: x={x}, y={y}, yaw={yaw}")
        self.nav_client.send_goal(goal)
        success = self.nav_client.wait_for_result(timeout=rospy.Duration(timeout))

        return success, self.nav_client.get_state()

    def execute_cb(self, goal):
        """
        Action server callback: Navigate through a sequence of coordinates.
        """
        rospy.loginfo(f"Received navigation goal: x={goal.x}, y={goal.y}, yaw={goal.yaw}")
        result = NavigateActionResult()

        # Determine the path from current position to target
        current_pose, current_quat = self.listener.lookupTransform("map", "base_footprint", rospy.Time(0))
        current_yaw = tf.transformations.euler_from_quaternion(current_quat)[2]
        xyth = np.asarray((current_pose[0], current_pose[1], current_yaw))

        # Quantize the current pose and target pose
        _, current_quantized = quantized(xyth, self.centroids)
        _, target_quantized = quantized(np.asarray((goal.x, goal.y, goal.yaw)), self.centroids)

        # Compute the path using Dijkstra or any other graph traversal
        path = []
        if current_quantized != target_quantized:
            path = dijkstra(current_quantized, target_quantized, Markov_A_2_grafo(A, self.centroids))

        rospy.loginfo(f"Computed path: {path}")
        if not path:
            rospy.logerr("No path found to the target. Exiting...")
            result.result.success = 0
            self.hmm_nav_server.set_aborted(result.result)
            return

        # Follow the path sequentially
        for point_idx in path:
            x, y, yaw = self.centroids[point_idx]
            rospy.loginfo(f"Navigating to path point: x={x}, y={y}, yaw={yaw} , path={path}, point_idx={point_idx}")
            success, state = self.send_move_base_goal(x, y, yaw)

            if not success or state != actionlib.GoalStatus.SUCCEEDED:
                rospy.logerr(f"Failed to reach waypoint at x={x}, y={y}, yaw={yaw}. Exiting...")
                result.result.success = 0
                self.hmm_nav_server.set_aborted(result.result)
                return

        rospy.loginfo("Successfully navigated to the final target.")
        result.result.success = 1
        self.hmm_nav_server.set_succeeded(result.result)

if __name__ == "__main__":
    #rospy.init_node("hmm_nav_move_base_sequential_server")
    rospy.loginfo("HMM Nav MoveBase Sequential Server is up")
    server = HMMNavMoveBaseServer()
    rospy.spin()

#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import moveit_msgs
import geometry_msgs.msg as geometry_msgs
import tf2_geometry_msgs
import shape_msgs
from math import pi
import tf
from trajectory_msgs.msg import JointTrajectoryPoint

def open_gripper(posture):
    posture.joint_names = ["hand_motor_joint"]
    posture.points = [JointTrajectoryPoint()]
    posture.points[0].positions = [0.04]
    posture.points[0].time_from_start = rospy.Duration(0.5)

def closed_gripper(posture):
    posture.joint_names = ["hand_motor_joint"]
    posture.points = [JointTrajectoryPoint()]
    posture.points[0].positions = [0.0]
    posture.points[0].time_from_start = rospy.Duration(0.5)

def pick(move_group):
    grasps = []
    grasps.append(moveit_msgs.msg.Grasp())

    # Setting grasp pose
    grasp_pose = geometry_msgs.PoseStamped()
    grasp_pose.header.frame_id = "odom"
    orientation = tf.transformations.quaternion_from_euler(-pi/4, -pi/8, -pi/4)
    grasp_pose.pose.orientation = geometry_msgs.Quaternion(*orientation)
    grasp_pose.pose.position.x = 0.415
    grasp_pose.pose.position.y = 0
    grasp_pose.pose.position.z = 0.5
    grasps[0].grasp_pose = grasp_pose

    # Setting pre-grasp approach
    pre_grasp_approach = moveit_msgs.msg.GripperTranslation()
    pre_grasp_approach.direction.header.frame_id = "odom"
    pre_grasp_approach.direction.vector.x = 1.0
    pre_grasp_approach.min_distance = 0.095
    pre_grasp_approach.desired_distance = 0.115
    grasps[0].pre_grasp_approach = pre_grasp_approach

    # Setting post-grasp retreat
    post_grasp_retreat = moveit_msgs.msg.GripperTranslation()
    post_grasp_retreat.direction.header.frame_id = "odom"
    post_grasp_retreat.direction.vector.z = 1.0
    post_grasp_retreat.min_distance = 0.1
    post_grasp_retreat.desired_distance = 0.25
    grasps[0].post_grasp_retreat = post_grasp_retreat

    # Setting posture of eef before grasp
    open_gripper(grasps[0].pre_grasp_posture)

    # Setting posture of eef during grasp
    closed_gripper(grasps[0].grasp_posture)

    # Set support surface as table1
    move_group.set_support_surface_name("table1")

    # Call pick to pick up the object using the grasps given
    move_group.pick("object", grasps[0])

def place(move_group):
    place_location = []
    place_location.append(moveit_msgs.msg.PlaceLocation())

    # Setting place location pose
    place_pose = geometry_msgs.PoseStamped()
    place_pose.header.frame_id = "odom"
    
    orientation = tf.transformations.quaternion_from_euler(0, 0, pi/4)
    place_pose.pose.orientation = geometry_msgs.Quaternion(*orientation)

    place_pose.pose.position.x = 0
    place_pose.pose.position.y = 0.5
    place_pose.pose.position.z = 0.5
    place_location[0].place_pose = place_pose.pose

    # Setting pre-place approach
    pre_place_approach = moveit_msgs.msg.GripperTranslation()
    pre_place_approach.direction.header.frame_id = "odom"
    pre_place_approach.direction.vector.z = -1.0
    pre_place_approach.min_distance = 0.095
    pre_place_approach.desired_distance = 0.115
    place_location[0].pre_place_approach = pre_place_approach

    # Setting post-place retreat
    post_place_retreat = moveit_msgs.msg.GripperTranslation()
    post_place_retreat.direction.header.frame_id = "odom"
    post_place_retreat.direction.vector.y = -1.0
    post_place_retreat.min_distance = 0.1
    post_place_retreat.desired_distance = 0.25
    place_location[0].post_place_retreat = post_place_retreat

    # Setting posture of eef after placing object
    open_gripper(place_location[0].post_place_posture)

    # Set support surface as table2
    move_group.set_support_surface_name("table2")

    # Call place to place the object using the place locations given
    move_group.place("object", place_location)

def add_collision_objects(scene):
    collision_objects = []

    # Add the first table where the cube will originally be kept.
    pose = geometry_msgs.PoseStamped()
    pose.header.frame_id = "odom"
    pose.pose.position.x = 0.5
    pose.pose.position.y = 0
    pose.pose.position.z = 0.2
    pose.pose.orientation.w = 1.0
    scene.add_box("table1", pose, size = [0.2, 0.4, 0.4])

    # Add the second table where we will be placing the cube.
    pose = geometry_msgs.PoseStamped()
    pose.header.frame_id = "odom"
    pose.pose.position.x = 0
    pose.pose.position.y = 0.5
    pose.pose.position.z = 0.2
    pose.pose.orientation.w = 1.0
    scene.add_box("table2", pose, size = [0.4, 0.2, 0.4])

    # Define the object that we will be manipulating
    pose = geometry_msgs.PoseStamped()
    pose.header.frame_id = "odom"
    pose.pose.position.x = 0.5
    pose.pose.position.y = 0
    pose.pose.position.z = 0.5
    pose.pose.orientation.w = 1.0
    scene.add_box("table2", pose, size = [0.02, 0.02, 0.2])

    #scene.add_collision_objects(collision_objects)

def main():
    rospy.init_node('hsrb_arm_pick_place', anonymous=True)
    rospy.sleep(1.0)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("whole_body_weighted")
    group.set_planning_time(45.0)
    scene = moveit_commander.PlanningSceneInterface()

    add_collision_objects(scene)

    rospy.sleep(1.0)

    pick(group)

    rospy.sleep(1.0)

    place(group)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

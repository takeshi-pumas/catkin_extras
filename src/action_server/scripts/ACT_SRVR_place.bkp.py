#!/usr/bin/env python3
import rospy
import sys
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import smach
from smach_ros import ActionServerWrapper , IntrospectionServer
import moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped, PoseStamped, TransformStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_commander import Constraints
from moveit_msgs.msg import OrientationConstraint
from action_server.msg import GraspAction
from std_srvs.srv import Empty

from tf.transformations import euler_from_quaternion, quaternion_from_euler , quaternion_multiply

from utils import grasp_utils

global clear_octo_client

clear_octo_client = rospy.ServiceProxy('/clear_octomap', Empty)
#from utils.grasp_utils import *

class PlacingStateMachine:
    def __init__(self):
        # Import gripper controller
        self.gripper = grasp_utils.GRIPPER()
        self.brazo = grasp_utils.ARM()
        self.base = grasp_utils.BASE()
        self.head = grasp_utils.GAZE()

        # Inicializar tf2_ros
        self.tf2_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf2_buffer)
        #self.br = tf2_ros.StaticTransformBroadcaster()

        # Inicializar MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.whole_body = moveit_commander.MoveGroupCommander("whole_body_weighted")
        #self.base = moveit_commander.MoveGroupCommander("base")
        #self.whole_body_w = moveit_commander.MoveGroupCommander("whole_body_weighted")
        #self.arm = moveit_commander.MoveGroupCommander("arm")
        self.grasp_approach = "frontal" #above / frontal
        self.eef_link = self.whole_body.get_end_effector_link()
        self.approach_limit = 10
        self.approach_count = 0

        # Moveit setup
        self.scene.remove_attached_object(self.eef_link, name="objeto")
        self.whole_body.allow_replanning(False)
        self.whole_body.set_num_planning_attempts(3)
        self.whole_body.set_planning_time(10.0)
        self.planning_frame = self.whole_body.get_planning_frame()
        
        # Crear la máquina de estados SMACH
        self.sm = smach.StateMachine(outcomes=['succeeded', 'failure', 'poured'],
                                     input_keys=["goal"])
        sis = IntrospectionServer('SMACH_VIEW_SERVER', self.sm, '/PLACE ACTION')
        sis.start()
        with self.sm:
            smach.StateMachine.add('CREATE_BOUND', smach.CBState(self.create_bound, outcomes=['success', 'failed']),
                                   transitions={'success':'APPROACH', 'failed':'CREATE_BOUND'})
            smach.StateMachine.add('APPROACH', smach.CBState(self.approach, outcomes=['success', 'failed', 'cancel','poured']),
                                   transitions={'success':'RETREAT', 'failed':'failure', 'cancel':'failure','poured':'succeeded' })
            
            smach.StateMachine.add('RETREAT', smach.CBState(self.retreat, outcomes=['success', 'failed']),
                                   transitions={'success':'succeeded', 'failed': 'RETREAT'})
            smach.StateMachine.add('NEUTRAL_POSE', smach.CBState(self.neutral_pose, outcomes=['success', 'failed']),
                        transitions={'success':'succeeded', 'failed': 'NEUTRAL_POSE'})

        self.wrapper = ActionServerWrapper("place_server", GraspAction, #from grasp.act
                                           wrapped_container = self.sm,
                                           succeeded_outcomes=["succeeded"],
                                           aborted_outcomes=["failed"],
                                           preempted_outcomes=["preempted"],
                                           goal_key='goal',
                                           result_key="action_done")
        self.wrapper.run_server()




    # SMACH states ------------------------------------------------------
    def create_bound(self, userdata):
        #self.add_collision_object('bound_left', position=[0.0, 0.6, 0.3], dimensions=[2.0, 0.05, 0.05])
        #self.add_collision_object('bound_right', position=[0.0, - 1.0, 0.3], dimensions=[2.0, 0.05, 0.05])
        #self.add_collision_object('bound_behind', position=[-0.4, 0.0, 0.3], dimensions=[0.05, 2.0, 0.05])
        self.publish_known_areas()# Add Table
        #clear_octo_client()        
        self.safe_pose = self.whole_body.get_current_joint_values()
        return 'success'


    def approach(self, userdata):
        #Add primitive objets to planning scene

        
        # Maybe create a safe area to plan... COnstraints .. maybe create positional constraint ( see pick service)
        
        self.approach_count += 1
        if self.approach_limit == self.approach_count:
            return 'cancel'
        goal = self.sm.userdata.goal.target_pose.data
        pos = [goal[0], goal[1], goal[2]]
        self.add_collision_object(position = [0,0,0.15], dimensions = [0.1, 0.1, 0.051], frame='hand_palm_link')
        self.attach_object()
        
        rospy.sleep(1)
        print(f'self.sm.userdata.goal -> {self.sm.userdata.goal.target_pose.data}')
        print (f'self.sm.userdata.goal.mode -> {self.sm.userdata.goal.mode.data}')
        self.grasp_approach=self.sm.userdata.goal.mode.data
        pose_goal = [goal[0], goal[1], goal[2], goal[3], goal[4], goal[5], goal[6]]
        if self.grasp_approach == "above":
            self.target_pose, gaze_dir = self.calculate_above_approach(target_position=pose_goal)
            print(f"Mode Above {self.target_pose}")
        else: #if self.grasp_approach == "frontal":
            self.target_pose = self.calculate_frontal_approach(target_position=pose_goal)
            print(self.target_pose)
        rospy.sleep(0.5)
        

        try:
            transform = self.tf2_buffer.lookup_transform("odom", "base_link", rospy.Time(0), timeout=rospy.Duration(1))
            rotation = transform.transform.rotation
            fixed_quat = [rotation.x, rotation.y, rotation.z, rotation.w]
            self.set_orientation_constraint(fixed_quat=fixed_quat)
            # Set orientation constraint based on the current orientation of base_link
            #self.set_combined_constraints(fixed_quat=fixed_quat)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Could not get transform from 'odom' to 'base_link'. Using default quaternion.")
            
        # Calculate the target pose based on the grasp approach

        #if self.grasp_approach == "above":
        #    self.target_pose, gaze_dir = self.calculate_above_approach(target_position=pose_goal)
        #else:
        #    self.target_pose = self.calculate_frontal_approach(target_position=pose_goal)
        
        rospy.sleep(0.5)

        ## ADD constraints for pouring ( Placing may not need them) #FOR POURING
        if self.sm.userdata.goal.mode.data== 'pour':
            try:
                transform = self.tf2_buffer.lookup_transform("odom", "base_link", rospy.Time(0), timeout=rospy.Duration(1))
                rotation = transform.transform.rotation
                fixed_quat = [rotation.x, rotation.y, rotation.z, rotation.w]
                # Set orientation constraint based on the current orientation of base_link
                self.set_orientation_constraint(fixed_quat=fixed_quat)
            
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("Could not get transform from 'odom' to 'base_link'. Using default quaternion.")
                # Use a default quaternion if unable to retrieve the current orientation
                #fixed_quat = [0, 0, 0, 1]
                #self.set_orientation_constraint(fixed_quat=fixed_quat)
        # Move the robot to the target pose with orientation constraint
        succ = self.move_to_pose(self.whole_body, self.target_pose)
        print (succ)
        rospy.sleep(3)
        # Clear the orientation constraint after the movement
        self.clear_orientation_constraint()
       
        ##ASK FOR POUR
        if succ:
            if self.grasp_approach== 'pour':                                                                                #POURING
                joint_values = self.brazo.get_joint_values()
                #safe_jv=joint_values
                joint_values[0] += -0.05
                joint_values[2] = 0.0
                joint_values[3] = 0.0
                joint_values[4] = np.pi
                self.brazo.set_joint_values(joint_values)     
                rospy.sleep(3.0)     
                joint_values[0] += 0.1
                joint_values[4] = 0.0
                self.brazo.set_joint_values(joint_values)     
                #self.brazo.set_joint_values(safe_jv)     
                rospy.sleep(0.5)    
                self.scene.remove_attached_object(self.eef_link, name="objeto") 
                return 'poured' 
            else:
                
                return 'success'
        else:
            return 'failed'

    

    def retreat(self, userdata):
        

        
        
        # first place then retreat.
            
        pose_eef = self.whole_body.get_current_pose()
        height_eef=pose_eef.pose.position.z
        goal = self.sm.userdata.goal.target_pose.data
        joint_values = self.brazo.get_joint_values()
        joint_values[0] +=  (goal[2]-height_eef)         #-0.1
        self.brazo.set_joint_values(joint_values)
        self.gripper.open()
        rospy.sleep(1.0)
        self.scene.remove_attached_object(self.eef_link, name="objeto")
        
        
        
        ### 
        ###TODO CREATE A SAFE RETREAT POSE!
        joint_values = self.brazo.get_joint_values()
        
        joint_values[0] += 0.2
        joint_values[1] += 0.2
        self.brazo.set_joint_values(joint_values)     

        rospy.sleep(1.0)
        return 'success'
        # if succ:
        #     return 'success'
        # else:
        #     return 'failed'
    
    def neutral_pose(self, userdata):
        self.brazo.set_named_target('neutral')
        # self.whole_body.go()
        self.scene.remove_world_object('bound_left')
        self.scene.remove_world_object('bound_right')
        self.scene.remove_world_object('bound_behind')
        self.scene.remove_world_object('objeto')
        return "success"
    
    # ----------------------------------------------------------
     # Add the orientation constraint
    def set_orientation_constraint(self, fixed_quat =[0, 0, 0, 1]):
        orientation_constraint = OrientationConstraint()
        orientation_constraint.link_name = "base_link"
        orientation_constraint.header.frame_id = "odom"
        orientation_constraint.orientation.x = fixed_quat[0]
        orientation_constraint.orientation.y = fixed_quat[1]
        orientation_constraint.orientation.z = fixed_quat[2]
        orientation_constraint.orientation.w = fixed_quat[3]
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.2
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.orientation_constraints.append(orientation_constraint)
        self.whole_body.set_path_constraints(constraints)

    # Clear the orientation constraint
    def clear_orientation_constraint(self):
        self.whole_body.clear_path_constraints()

###################################################################################################        
    def move_to_position(self, group, position_goal):
        group.set_start_state_to_current_state()
        group.set_position_target(position_goal)
        succ = group.go(wait= True)
        rospy.sleep(0.5)
        group.stop()
        return succ

    def move_to_pose(self, group, pose_goal):
        group.set_start_state_to_current_state()
        group.set_pose_target(pose_goal)
        succ = group.go(wait=True)
        rospy.sleep(0.5)
        group.stop()
        return succ
                                        #position = [6.2, 1.1, 0.36], rotation = [0,0,0.707,0.707]#Table breakfast
                                        #position = [4.5, 3.0, 0.35], rotation = [0,0,0,1]#Table pickup
    def publish_known_areas(self, position =     [-0.7, -3.1, 0.75], rotation = [0,0,0.0,1], dimensions = [3.0 ,1.0, 0.02]): #position = [5.9, 5.0,0.3] ##SIM
                                                                                                                   #position = [-0.7, -3.3, 0.75]###REAL
                                                                                                                   #position =[4.5, 3.0, 0.4] ### TMR
    
        object_pose = PoseStamped()
        object_pose.header.frame_id = 'map'
        object_pose.pose.position.x = position[0]
        object_pose.pose.position.y = position[1]
        object_pose.pose.position.z = position[2]
        object_pose.pose.orientation.x = rotation[0]
        object_pose.pose.orientation.y = rotation[1]
        object_pose.pose.orientation.z = rotation[2]
        object_pose.pose.orientation.w = rotation[3]
        #self.scene.add_box('table_storing', object_pose, size = (dimensions[0], dimensions[1], dimensions[2]))

    def add_collision_object(self, name = 'objeto', position = [0, 0, 0], rotation = [0,0,0,1], dimensions = [0.3 ,0.3, 0.3], frame = 'base_link'):
        object_pose = PoseStamped()
        object_pose.header.frame_id = frame
        object_pose.pose.position.x = position[0]
        object_pose.pose.position.y = position[1]
        object_pose.pose.position.z = position[2]
        object_pose.pose.orientation.x = rotation[0]
        object_pose.pose.orientation.y = rotation[1]
        object_pose.pose.orientation.z = rotation[2]
        object_pose.pose.orientation.w = rotation[3]
        self.scene.add_box(name, object_pose, size = (dimensions[0], dimensions[1], dimensions[2]))

    def attach_object(self):

        grasping_group = "arm"
        touch_links = self.robot.get_link_names(group=grasping_group)

        self.scene.attach_box(self.eef_link, "objeto", touch_links=touch_links)

    def execute_cb(self, goal):
        rospy.loginfo('Received action goal: %s', goal)
        self.sm.userdata.goal = goal
        
        distance = np.linalg.norm(goal)
        if len(goal) == 3 and distance < 1.5:
            self.wrapper.server.set_succeeded()
            outcome = self.sm.execute()
        else:
            rospy.loginfo("Goal not valid")

    def calculate_frontal_approach(self, target_position = [0.0, 0.0, 0.0]):
        object_point = PointStamped()
        object_point.header.frame_id = "odom"#"base_link"
        object_point.point.x = target_position[0]
        object_point.point.y = target_position[1]
        object_point.point.z = target_position[2]
        
        #transformar la posicion del objeto al marco de referencia de la base del robot
        try:
            transformed_object_point = self.tf2_buffer.transform(object_point, "odom", timeout=rospy.Duration(1))
            transformed_base = self.tf2_buffer.lookup_transform("odom", "base_link", rospy.Time(0), timeout=rospy.Duration(1))
            
        except :
            rospy.WARN("Error al transformar la posicion del objeto al marco de referencia")
            return None, None
        
        approach_pose = Pose()
        approach_pose.position.x = transformed_object_point.point.x 
        approach_pose.position.y = transformed_object_point.point.y
        approach_pose.position.z = transformed_object_point.point.z +0.1

        quat_base = [transformed_base.transform.rotation.x,
                               transformed_base.transform.rotation.y,
                               transformed_base.transform.rotation.z,
                               transformed_base.transform.rotation.w]
        
        quat_reqgoal=[target_position[3],target_position[4],target_position[5],target_position[6]]  # Quaternion added to pose  act request
        rot_quat=quaternion_multiply(quat_reqgoal, [0,0,1,0])
        _,_,theta = euler_from_quaternion(rot_quat)        
        quat = quaternion_from_euler(0.0, -0.5*np.pi, theta , axes='sxyz')
        approach_pose.orientation = Quaternion(*quat)
        
        return approach_pose
    
    def calculate_above_approach(self, target_position = [0.0, 0.0, 0.0]):
        object_point = PointStamped()
        object_point.header.frame_id = "odom"
        object_point.point.x = target_position[0]
        object_point.point.y = target_position[1]
        object_point.point.z = target_position[2]
        
        #transformar la posicion del objeto al marco de referencia de la base del robot
        try:
            transformed_object_point = self.tf2_buffer.transform(object_point, "odom", timeout=rospy.Duration(1))
            transformed_object_base = self.tf2_buffer.transform(object_point, "base_link", timeout=rospy.Duration(1))
            transformed_base = self.tf2_buffer.lookup_transform("odom", "base_link", rospy.Time(0), timeout=rospy.Duration(1))
            transformed_object_base.point.x += 0.02
            transformed_object_point = self.tf2_buffer.transform(transformed_object_base, "odom", timeout=rospy.Duration(1))
        except :
            rospy.WARN("Error al transformar la posicion del objeto al marco de referencia")
            return None
        approach_pose = Pose()
        approach_pose.position.x = transformed_object_point.point.x
        approach_pose.position.y = transformed_object_point.point.y
        approach_pose.position.z = transformed_object_point.point.z + 0.24

        quat_base = [transformed_base.transform.rotation.x,
                               transformed_base.transform.rotation.y, 
                               transformed_base.transform.rotation.z,
                               transformed_base.transform.rotation.w]
        _,_,theta = euler_from_quaternion(quat_base)
        quat = quaternion_from_euler(-theta, 0.0, np.pi, 'szyx')
        approach_pose.orientation = Quaternion(*quat)
        
        return approach_pose, transformed_object_base
if __name__ == '__main__':
    rospy.init_node('placing_action')
    placing_sm = PlacingStateMachine()
    #rospy.spin()

import rospy
import sys
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import smach
from smach_ros import ActionServerWrapper
import moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped, PoseStamped, TransformStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from action_server.msg import GraspAction

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from utils.grasp_utils import *

class GraspingStateMachine:
    def __init__(self):
        # Import gripper controller
        self.gripper = GRIPPER()

        # Inicializar tf2_ros
        self.tf2_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.br = tf2_ros.StaticTransformBroadcaster()
        

        # Inicializar MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.whole_body = moveit_commander.MoveGroupCommander("whole_body")
        #self.whole_body_w = moveit_commander.MoveGroupCommander("whole_body_weighted")
        #self.arm = moveit_commander.MoveGroupCommander("arm")
        self.grasp_approach = "above" #above / frontal
        self.eef_link = self.whole_body.get_end_effector_link()

        # Moveit setup
        #self.scene.remove_attached_object(self.eef_link, name="objeto")
        self.whole_body.allow_replanning(True)
        self.whole_body.set_num_planning_attempts(10)
        self.whole_body.set_planning_time(10.0)
        #self.whole_body_w.allow_replanning(True)
        #self.whole_body_w.set_num_planning_attempts(10)
        #self.whole_body_w.set_planning_time(10.0)
        self.whole_body.set_workspace([-2.0, -2.0, 0.0, 2.0, 2.0, 2.0])
        #self.whole_body_w.set_workspace([-2.0, -2.0, 2.0, 2.0])
        self.planning_frame = self.whole_body.get_planning_frame()
        print(self.planning_frame)
        self.gripper.open()

        # Definir la posición objetivo (cambiar según necesidad)
        self.target_pose = Pose()
        self.target_pose.position.x = 0.8
        self.target_pose.position.y = 0.2
        self.target_pose.position.z = 0.72
        self.target_pose.orientation.w = 1.0

        # Crear la máquina de estados SMACH
        self.sm = smach.StateMachine(outcomes=['success', 'failure'])
        with self.sm:
            smach.StateMachine.add('APPROACH', smach.CBState(self.approach, outcomes=['success']),
                                   transitions={'success':'GRASP'})
            smach.StateMachine.add('GRASP', smach.CBState(self.grasp, outcomes=['success']),
                                   transitions={'success':'RETREAT'})
            smach.StateMachine.add('RETREAT', smach.CBState(self.retreat, outcomes=['success']),
                                   transitions={'success':'success'})

        # Inicializar el servidor de acción
        #self.server = SimpleActionServer('follow_server', FollowAction, execute_cb=self.execute_cb, auto_start=False)
        #self.server.start()
        self.wrapper = ActionServerWrapper("grasp_server", GraspAction,
                                           wrapped_container = self.sm,
                                           succeeded_outcomes=["succeeded"],
                                           aborted_outcomes=["failed"],
                                           preempted_outcomes=["preempted"],
                                           goal_key='goal',
                                           result_key="action_done")
        self.wrapper.run_server()

        outcome = self.sm.execute()

    def calculate_frontal_approach(self, target_position = [0.0, 0.0, 0.0]):
        object_point = PointStamped()
        object_point.header.frame_id = "base_link"
        object_point.point.x = target_position[0]
        object_point.point.y = target_position[1]
        object_point.point.z = target_position[2]

        #transformar la posicion del objeto al marco de referencia de la base del robot
        try:
            transformed_object_point = self.tf2_buffer.transform(object_point, "base_link", timeout=rospy.Duration(1))
            transformed_base = self.tf2_buffer.lookup_transform("odom", "base_link", rospy.Time(0), timeout=rospy.Duration(1))
        except :
            rospy.WARN("Error al transformar la posicion del objeto al marco de referencia")
            return None, None
        
        approach_pose = Pose()
        approach_pose.position.x = transformed_object_point.point.x - 0.12
        approach_pose.position.y = transformed_object_point.point.y
        approach_pose.position.z = transformed_object_point.point.z

        quat_base = [transformed_base.transform.rotation.x,
                               transformed_base.transform.rotation.y,
                               transformed_base.transform.rotation.z,
                               transformed_base.transform.rotation.w]
        _,_,theta = euler_from_quaternion(quat_base)
        quat = quaternion_from_euler(np.pi, -np.pi/2, -theta, axes='sxyx')
        approach_pose.orientation = Quaternion(*quat)
        return approach_pose
    
    def calculate_above_approach(self, target_position = [0.0, 0.0, 0.0]):
        object_point = PointStamped()
        object_point.header.frame_id = "base_link"
        object_point.point.x = target_position[0]
        object_point.point.y = target_position[1]
        object_point.point.z = target_position[2]

        #transformar la posicion del objeto al marco de referencia de la base del robot
        try:
            transformed_object_point = self.tf2_buffer.transform(object_point, "base_link", timeout=rospy.Duration(1))
            transformed_base = self.tf2_buffer.lookup_transform("odom", "base_link", rospy.Time(0), timeout=rospy.Duration(1))
        except :
            rospy.WARN("Error al transformar la posicion del objeto al marco de referencia")
            return None
        
        approach_pose = Pose()
        approach_pose.position.x = transformed_object_point.point.x
        approach_pose.position.y = transformed_object_point.point.y
        approach_pose.position.z = transformed_object_point.point.z + 0.12

        #print(transformed_base)
        quat_base = [transformed_base.transform.rotation.x,
                               transformed_base.transform.rotation.y,
                               transformed_base.transform.rotation.z,
                               transformed_base.transform.rotation.w]
        _,_,theta = euler_from_quaternion(quat_base)
        quat = quaternion_from_euler(-theta, 0.0, np.pi, 'szyx')
        approach_pose.orientation = Quaternion(*quat)

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "goal_pose"
        t.transform.translation = approach_pose.position
        t.transform.rotation = approach_pose.orientation
        self.br.sendTransform(t)

        return approach_pose

    # SMACH states ------------------------------------------------------
    def approach(self, userdata):
        #Add primitive objets to planning scene
        pos = [self.target_pose.position.x,
                self.target_pose.position.y,
                self.target_pose.position.z]
        self.add_collision_object(position = pos,dimensions = [0.05, 0.05, 0.05])

        self.gripper.open()
        #pose_goal = self.target_pose
        #approach_pose = pose_goal
        #approach_pose.position.z += 0.1  # 10 cm por encima de la pieza
        pose_goal = [self.target_pose.position.x, 
                     self.target_pose.position.y,
                     self.target_pose.position.z]
        if self.grasp_approach == "frontal":
            self.target_pose = self.calculate_frontal_approach(target_position=pose_goal)
            print(self.target_pose)
        elif self.grasp_approach == "above":
            self.target_pose = self.calculate_above_approach(target_position=pose_goal)
            print(self.target_pose)
        rospy.sleep(0.5)
        self.move_to_pose(self.whole_body, self.target_pose)
        return 'success'

    def grasp(self, userdata):
        
        #Plan to grasp object
        pose_goal = self.target_pose
        grasp_pose = pose_goal
        if self.grasp_approach == "frontal":
            grasp_pose.position.x += 0.05   # Adelantarse a la pieza
        elif self.grasp_approach == "above":
            grasp_pose.position.z -= 0.05  # Descender hacia la pieza
        self.move_to_pose(self.whole_body, grasp_pose)
        self.gripper.close(0.03)
        #self.attach_object()
        return 'success'

    def retreat(self, userdata):
        pose_goal = self.target_pose
        #pose_goal.position.x -= 0.13
        pose_goal.position.z += 0.13
        position_goal = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z]
        self.move_to_position(self.whole_body, position_goal)  # Retirarse a una posición segura
        return 'success'
    
    def neutral_pose(self, userdata):
        self.whole_body.set_named_target("neutral")
        self.whole_body.go()
        return "success"
    
    # ----------------------------------------------------------
    def move_to_position(self, group, position_goal):
        group.set_start_state_to_current_state()
        group.set_position_target(position_goal)
        plan = group.go(wait= True)
        rospy.sleep(0.5)
        group.stop()

    def move_to_pose(self, group, pose_goal):
        group.set_start_state_to_current_state()
        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)
        rospy.sleep(0.5)
        group.stop()

    def add_collision_object(self, position = [0, 0, 0], dimensions = [0.1 ,0.1, 0.1]):
        # collision_object = CollisionObject()
        # collision_object.id = "objeto"
        # collision_object.header.frame_id = "map" #self.group.get_planning_frame()
        # primitive = SolidPrimitive()
        # primitive.type = SolidPrimitive.BOX
        # primitive.dimensions = dimensions
        # pose = Pose()
        # pose.position.x = position[0]
        # pose.position.y = position[1]
        # pose.position.z = position[2]
        # collision_object.primitives.append(primitive)
        # collision_object.primitive_poses.append(pose)
        # self.scene.add_object(collision_object)
        object_pose = PoseStamped()
        object_pose.header.frame_id = self.whole_body.get_planning_frame()
        object_pose.pose = self.target_pose
        self.scene.add_box("objeto", object_pose, size = (dimensions[0], dimensions[1], dimensions[2]))

    def attach_object(self):

        grasping_group = "arm"
        touch_links = self.robot.get_link_names(group=grasping_group)

        self.scene.attach_box(self.eef_link, "objeto", touch_links=touch_links)
        # attached_object = AttachedCollisionObject()
        # attached_object.object.id = "objeto"
        # attached_object.link_name = "hand_palm_link"

        # attached_object.object.operation = attached_object.object.ADD

        # self.scene.attach_object(attached_object)

    def execute_cb(self, goal):
        rospy.loginfo('Received action goal: %s', goal)
        self.sm.userdata.goal = goal
        self.wrapper.server.set_succeeded()
        outcome = self.sm.execute()
        # result = FollowResult()
        # result.success = True if outcome == 'success' else False
        # self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('grasping_demo')
    grasping_sm = GraspingStateMachine()
    #rospy.spin()

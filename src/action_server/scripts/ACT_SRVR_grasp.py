import rospy
import sys
import smach
from smach_ros import ActionServerWrapper
import moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
from action_server.msg import GraspAction
#from actionlib import SimpleActionServer
#from your_package_name.msg import FollowAction, FollowGoal, FollowResult, FollowFeedback

class GraspingStateMachine:
    def __init__(self):
        # Inicializar MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "whole_body"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        # Definir la posición objetivo (cambiar según necesidad)
        self.target_pose = Pose()
        self.target_pose.position.x = 0.5
        self.target_pose.position.y = 0.5
        self.target_pose.position.z = 0.5
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

    def approach(self, userdata):
        pose_goal = self.target_pose
        approach_pose = pose_goal
        approach_pose.position.z += 0.1  # 10 cm por encima de la pieza
        self.move_to_pose(approach_pose)
        return 'success'

    def grasp(self, userdata):
        pose_goal = self.target_pose
        grasp_pose = pose_goal
        grasp_pose.position.z -= 0.05  # Descender hacia la pieza
        self.move_to_pose(grasp_pose)
        # Agregar la lógica para cerrar la pinza aquí
        self.add_collision_object(0.1, 0.1, 0.1)  # Tamaño aproximado de la pieza
        rospy.sleep(1)
        return 'success'

    def retreat(self, userdata):
        pose_goal = self.target_pose
        self.move_to_pose(pose_goal)  # Retirarse a una posición segura
        return 'success'

    def move_to_pose(self, pose_goal):
        self.group.set_pose_target(pose_goal)
        plan = self.group.go(wait=True)
        rospy.sleep(1)

    def add_collision_object(self, x, y, z):
        collision_object = CollisionObject()
        collision_object.id = "objeto"
        collision_object.header.frame_id = self.group.get_planning_frame()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [x, y, z]
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        self.scene.add_object(collision_object)

    def execute_cb(self, goal):
        rospy.loginfo('Received action goal: %s', goal)
        self.sm.userdata.goal = goal
        self.wrapper.server.set_succeeded()
        #outcome = self.sm.execute()
        # result = FollowResult()
        # result.success = True if outcome == 'success' else False
        # self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('grasping_demo')
    grasping_sm = GraspingStateMachine()
    rospy.spin()

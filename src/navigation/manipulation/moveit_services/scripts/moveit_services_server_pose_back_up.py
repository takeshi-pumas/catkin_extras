#!/usr/bin/env python

from moveit_services.srv import moveitPosition, moveitPick, moveitPlace 
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import tf

global robot


class TakeshiMoveIt():
	def __init__(self, wait=0.0):
		#Initialization
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.reference_frame = "odom"
		self.arm = moveit_commander.MoveGroupCommander("arm")
		self.base = moveit_commander.MoveGroupCommander("base")
		self.gripper = moveit_commander.MoveGroupCommander("gripper")
		self.head = moveit_commander.MoveGroupCommander("head")
		self.whole_body = moveit_commander.MoveGroupCommander("whole_body")
		self.whole_body_light = moveit_commander.MoveGroupCommander("whole_body_light")
		self.whole_body_weighted = moveit_commander.MoveGroupCommander("whole_body_weighted")
		self.whole_body.allow_replanning(True)
		self.whole_body.set_planning_time(5)
		self.whole_body.set_pose_reference_frame(self.reference_frame)
		self.end_effector = self.whole_body.get_end_effector_link()

		rospy.sleep(1)
		print self.end_effector
		# remove all objects
		self.scene.remove_attached_object(self.end_effector)
		self.scene.remove_world_object()
		rospy.sleep(1)

		# move_to_neutral
		rospy.loginfo("step1: go")
		self.base.go()
		self.arm.set_named_target("go")
		self.arm.go()
		self.head.set_named_target("neutral")
		self.head.go()
		self.gripper.set_joint_value_target("hand_motor_joint", 0.5)
		self.gripper.go()
		rospy.logdebug("done")
		rospy.sleep(wait)
		self.listener = tf.TransformListener()

	########################
	####move to position####	
	########################

	def go_to_pose_goal(self, move_group, pose_goal):
		print "moveit_services trying to reach pose:"
		print pose_goal
		commander = robot.chooseMoveGroup(move_group)
		#pose_goal=commander.get_random_pose()
		commander.set_pose_target(pose_goal.pose)
		succed = commander.go(wait=True)
		commander.stop()
		commander.clear_pose_targets()
		return succed

	def chooseMoveGroup(self,move_group):
		if move_group=="arm":
			return robot.arm
		elif move_group=="whole_body":
			return robot.whole_body
		elif move_group=="whole_body_light":
			return robot.whole_body_light
		elif move_group=="whole_body_weighted":
			return robot.whole_body_weighted
		else:
			print "move_group not valid default: whole_body"
			return robot.whole_body

	###########################
	###Pick and place object###
	###########################
	
	def pick(self, target, grasps):
		n_attempts = 0
		max_pick_attempts = 10
		result = None

		while (result != moveit_msgs.msg.MoveItErrorCodes.SUCCESS) and \
			  (n_attempts < max_pick_attempts):
			n_attempts += 1
			rospy.loginfo("Pick attempt: " + str(n_attempts))
			result = self.whole_body.pick(target, grasps)
			rospy.sleep(0.2)
		if result != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
			self.scene.remove_attached_object(self.end_effector)
		return result

	def place(self, target, location):
		n_attempts = 0
		max_pick_attempts = 10
		result = None

		while (result != moveit_msgs.msg.MoveItErrorCodes.SUCCESS) and \
			  (n_attempts < max_pick_attempts):
			n_attempts += 1
			rospy.loginfo("Place attempt: " + str(n_attempts))
			result = self.whole_body.place(target, location)
			rospy.sleep(0.2)
		if result != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
			self.scene.remove_attached_object(self.end_effector)
		return result

	def make_gripper_posture(self, pos, effort=0.0):
		t = trajectory_msgs.msg.JointTrajectory()
		t.joint_names = ["hand_motor_joint"]
		tp = trajectory_msgs.msg.JointTrajectoryPoint()
		tp.positions = [pos]
		tp.effort = [effort]
		tp.time_from_start = rospy.Duration(2.0)
		t.points.append(tp)
		return t

	def make_gripper_translation(self, min_dist, desired, vector, frame=None):
		g = moveit_msgs.msg.GripperTranslation()
		g.direction.vector.x = vector[0]
		g.direction.vector.y = vector[1]
		g.direction.vector.z = vector[2]
		if frame is None:
			g.direction.header.frame_id = self.end_effector
		else:
			g.direction.header.frame_id = frame
		g.min_distance = min_dist
		g.desired_distance = desired
		return g

	def make_pose(self, init, x, y, z, roll, pitch, yaw):
		pose = geometry_msgs.msg.PoseStamped()
		pose.header.frame_id = self.reference_frame
		q = quaternion_from_euler(roll, pitch, yaw)
		q = quaternion_multiply(init, q)
		pose.pose.orientation.x = q[0]
		pose.pose.orientation.y = q[1]
		pose.pose.orientation.z = q[2]
		pose.pose.orientation.w = q[3]
		pose.pose.position.x = x
		pose.pose.position.y = y
		pose.pose.position.z = z
		return pose

	def make_grasps(self, target, init,
					quality=None,
					x=[0], y=[0], z=[0],
					roll=[0], pitch=[0], yaw=[0]):
		poses = self.scene.get_object_poses([target])
		pose = poses[target]
		g = moveit_msgs.msg.Grasp()
		g.pre_grasp_posture = self.make_gripper_posture(0.8)
		g.grasp_posture = self.make_gripper_posture(0.2, -0.01)
		g.pre_grasp_approach \
			= self.make_gripper_translation(0.01, 0.02, [0.0, 0.0, 1.0])
		g.post_grasp_retreat \
			= self.make_gripper_translation(0.01, 0.02, [0.0, 0.0, 1.0],
											"base_footprint")
		grasps = []
		for ix in x:
			for iy in y:
				for iz in z:
					for iroll in roll:
						for ipitch in pitch:
							for iyaw in yaw:
								x = pose.position.x + ix
								y = pose.position.y + iy
								z = pose.position.z + iz
								g.grasp_pose = self.make_pose(init,
															  x, y, z,
															  iroll,
															  ipitch,
															  iyaw)
			g.id = str(len(grasps))
			g.allowed_touch_objects = ["target1"]
			g.max_contact_force = 0
			if quality is None:
				g.grasp_quality = 1.0
			else:
				g.grasp_quality = quality(ix, iy, iz, iroll, ipitch, iyaw)
			grasps.append(deepcopy(g))
		return grasps

	def make_place_location(self, x, y, z):
		location = moveit_msgs.msg.PlaceLocation()
		location.pre_place_approach \
			= self.make_gripper_translation(0.03, 0.05, [0, 0, -1.0],
											"base_footprint")
		location.post_place_posture \
			= self.make_gripper_posture(0.8)
		location.post_place_retreat \
			= self.make_gripper_translation(0.03, 0.05, [0, 0, -1.0])
		location.place_pose = self.make_pose((0, 0, 0, 1),
											 x, y, z,
											 0, 0, 0)
		return location

	def transform_to_reference_frame(self, pose):
		try:
			(trans,rot) = self.listener.lookupTransform(self.reference_frame,pose.header.frame_id, rospy.Time(0))
			print "Original Pose:"
			print pose
			print"#########################"
			print"tranfrom"
			print"#########################"
			print trans
			print rot

			print"#########################"
			print"applaying transform"
			print"#########################"
			pose.pose.position.x+=trans[0]
			pose.pose.position.y+=trans[1]
			pose.pose.position.z+=trans[2]

			pose.pose.orientation.x+=rot[0]
			pose.pose.orientation.y+=rot[1]
			pose.pose.orientation.z+=rot[2]
			pose.pose.orientation.w+=rot[3]
			pose.header.stamp=rospy.Time()
			pose.header.frame_id=self.reference_frame
			print"#########################"
			print"transformed pose"
			print"#########################"
			print pose
			return pose
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print"Error: Something went wrong with the tranfromation in moviet_serce.py"
			return None


######################
#### ROS services ####
######################

def handle_moveit_position(req):
	global robot 

	#return robot. go_to_pose_goal(req.move_group,req.pose)
	if(req.pose.header.frame_id==robot.reference_frame or req.pose.header.frame_id=="" ):
		return robot.go_to_pose_goal(req.move_group,req.pose)
	else:
		return robot. go_to_pose_goal(req.move_group,robot.transform_to_reference_frame(req.pose))

def handle_pick(req):
	global robot 
	
	return False

def handle_place(req):
	global robot 
	
	return False

def main():
	global robot
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('moveit_services_server',anonymous=True)
	robot=TakeshiMoveIt()
	s_goal_position = rospy.Service('/hsrb/manipulation/moveit/goal_position', moveitPosition, handle_moveit_position)
	s_pick = rospy.Service('/hsrb/manipulation/moveit/pick', moveitPosition, handle_pick)
	s_place = rospy.Service('/hsrb/manipulation/moveit/place', moveitPlace, handle_place)
	print("MOVEIT_SERVICES_SERVER INITIALIZED by HUGUIN SANCHEZ")

	while not rospy.is_shutdown():
		rospy.spin()

if __name__ == "__main__":
	main()



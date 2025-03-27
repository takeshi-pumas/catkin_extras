#!/usr/bin/env python3
import rospy
import smach
from common.states import WaitPushHand, WaitDoorOpen
from common.hsr_functions import Talker, Gaze
from common.ros_functions import TFManager


if __name__ == '__main__':
    print("Testing Common states")
    rospy.init_node("test_common_states")
    voice = Talker()
    tf_manager = TFManager()
    gaze = Gaze()

    # Publicar una transformación dinámica usando ángulos de Euler
    tf_manager.publish_transform(
        position=[1.0, 0.0, 1.0],
        rotation=[0.0, 0.0, 0.0],  # roll, pitch, yaw
        child_frame="object1",
        parent_frame="map",
        static = True
    )
    rospy.sleep(1.0)

    tf_manager.publish_transform(
        position=[1.0, 1.0, 1.0],
        rotation=[0.0, 0.0, 0.0],  # roll, pitch, yaw
        child_frame="object2",
        parent_frame="map",
        static = True
    )
    rospy.sleep(1.0)

    tf_manager.publish_transform(
        position=[1.0, -1.0, 1.0],
        rotation=[0.0, 0.0, 0.0],  # roll, pitch, yaw
        child_frame="object3",
        parent_frame="map",
        static = True
    )
    rospy.sleep(1.0)

    tf_manager.publish_transform(
        position=[1.0, 0.0, 0.0],
        rotation=[0.0, 0.0, 0.0],  # roll, pitch, yaw
        child_frame="object4",
        parent_frame="map",
        static = True
    )
    rospy.sleep(1.0)

    tf_manager.publish_transform(
        position=[1.0, 0.0, 2.0],
        rotation=[0.0, 0.0, 0.0],  # roll, pitch, yaw
        child_frame="object5",
        parent_frame="map",
        static = True
    )
    rospy.sleep(1.0)

    # Obtener una transformación
    # position, rotation = tf_manager.get_transform("map", "object")

    # Transformar una pose entre frames
    # new_pos, new_rot = tf_manager.transform_pose(
    #     position=[1.0, 0.0, 3.0],
    #     rotation=[0.0, 0.0, 0.0, 1.0],
    #     from_frame="object",
    #     to_frame="map"
    # )


    # Test different positions
    # positions = [
    #     (1.0, 0.0, 1.0),  # Front
    #     (1.0, 1.0, 1.0),  # Left
    #     (1.0, -1.0, 1.0), # Right
    #     (1.0, 0.0, 0.0),  # Down
    #     (1.0, 0.0, 2.0)   # Up
    # ]

    for i in range(1, 6):
        print(f"Looking at object{i}")
        gaze.look_at_frame(f"object{i}") #(*pos, frame='map')
        rospy.sleep(4.0)

    sm = smach.StateMachine(outcomes=['succ', 'failed'])






    # print("Posicion y rotacion del objeto ahora con respecto a map",new_pos, new_rot)

    with sm:
        smach.StateMachine.add('WAIT_PUSH_HAND', 
        WaitPushHand(talker = voice, talk_message=" ", timeout=1000), transitions={'succ': 'WAIT_DOOR_OPEN', 'failed': 'WAIT_PUSH_HAND', 'ended': 'failed'})
        smach.StateMachine.add('WAIT_DOOR_OPEN', 
        WaitDoorOpen(talker = voice, talk_message=" ", timeout=1000), transitions={'succ': 'succ', 'failed': 'WAIT_DOOR_OPEN', 'ended': 'failed'})
    outcome = sm.execute()
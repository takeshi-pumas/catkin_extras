#!/usr/bin/env python3
from common.states import WaitPushHand, WaitDoorOpen
from common.utils import Talker, TFManager
import rospy
import smach


if __name__ == '__main__':
    print("Testing Common states")
    rospy.init_node("test_common_states")
    voice = Talker()
    tf_manager = TFManager()
    sm = smach.StateMachine(outcomes=['succ', 'failed'])
    # voice.talk("Testing common states")

    # Ejemplo de uso
    tf_manager = TFManager()

    # Publicar una transformación dinámica usando ángulos de Euler
    tf_manager.publish_transform(
        position=[1.0, 0.0, 0.0],
        rotation=[0.0, 0.0, 0.0],  # roll, pitch, yaw
        child_frame="object",
        parent_frame="map",
        static = True
    )

    # Obtener una transformación
    position, rotation = tf_manager.get_transform("map", "object")

    # Transformar una pose entre frames
    new_pos, new_rot = tf_manager.transform_pose(
        position=[1.0, 0.0, 0.0],
        rotation=[0.0, 0.0, 0.0, 1.0],
        from_frame="object",
        to_frame="map"
    )

    print("Posicion y rotacion del objeto ahora con respecto a map",new_pos, new_rot)

    with sm:
        smach.StateMachine.add('WAIT_PUSH_HAND', 
        WaitPushHand(talker = voice, timeout=1000), transitions={'succ': 'WAIT_DOOR_OPEN', 'failed': 'WAIT_PUSH_HAND', 'ended': 'failed'})
        smach.StateMachine.add('WAIT_DOOR_OPEN', 
        WaitDoorOpen(talker = voice, timeout=1000), transitions={'succ': 'succ', 'failed': 'WAIT_DOOR_OPEN', 'ended': 'failed'})
    outcome = sm.execute()
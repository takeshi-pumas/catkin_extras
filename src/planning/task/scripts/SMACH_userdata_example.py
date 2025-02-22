#!/usr/bin/env python

import rospy
import smach
import smach_ros

class StateOne(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'], input_keys=['name'], output_keys=['name'])

    def execute(self, userdata):
        rospy.loginfo("Executing State One")
        rospy.loginfo("Name: %s", userdata.name)
        userdata.name = "Pedro"  # Modificamos el valor de name
        return 'success'

class StateTwo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'], input_keys=['name'])

    def execute(self, userdata):
        rospy.loginfo("Executing State Two")
        rospy.loginfo("Name: %s", userdata.name)
        return 'success'

def main():
    rospy.init_node('smach_example')

    # Creamos un objeto StateMachine
    sm = smach.StateMachine(outcomes=['finished'])

    sm.userdata.name = "Juan"

    # Definimos las transiciones y los estados
    with sm:
        smach.StateMachine.add('STATE_ONE', StateOne(), transitions={'success': 'STATE_TWO'},
                               remapping={'name': 'name'})
        smach.StateMachine.add('STATE_TWO', StateTwo(), transitions={'success': 'finished'},
                               remapping={'name': 'name'})

    # Creamos el planificador de estados
    outcome = sm.execute()

    rospy.loginfo("StateMachine executed with outcome: %s", outcome)

if __name__ == '__main__':
    main()

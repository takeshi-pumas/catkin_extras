import PySimpleGUI as sg
import tab_content
import functions
import roslaunch
import os
import rospy
from ROSnode import BASE_CONTROLLER

# Initial setup 
# Find workspace "catkin_extras"
functions.search_ws(folder="catkin_extras", source=True)
# os.environ['ROS_MASTER_URI'] = 'http://hsrb.local_et:11311'
# os.environ['ROS_IP'] = '169.254.2.172'

# roslaunch setup
global uuid
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

layout = tab_content.main_layout

window = sg.Window("ROS GUI", layout, size= tab_content.size_window)

if rospy.is_shutdown():
    rospy.init_node("ROS_GUI")
    base = BASE_CONTROLLER('topic')


while True:
    event, values = window.read()
    if event == sg.WINDOW_CLOSED:
        functions.cleanup_nodes()
        break
    elif event == '-CONNECT-':
        pass

    elif event == '-NAVIGATION-':
        functions.manage_node('navigation_real', 'nav_pumas', window_event=window[event], uuid=uuid)

    elif event == '-MOVEIT-':
        functions.manage_node('hsrb_demo_with_controller', 'hsrb_moveit_config', window_event=window[event], uuid=uuid)

    elif event == '-MOVEIT_ARM-':
        functions.manage_node('arm_test', 'task', window_event=window[event], uuid=uuid)
    
    elif event == '-FORWARD-':
        base.forward(values['-SLIDER-'])
        rospy.sleep(0.3)

    elif event == '-BACKWARD-':
        base.backward(values['-SLIDER-'])

    elif event == '-LEFT-':
        base.left(values['-SLIDER-'])

    elif event == '-RIGHT-':
        base.right(values['-SLIDER-'])

    elif event == '-TURN_L-':
        base.turn_l(values['-SLIDER-'])
    
    elif event == '-TURN_R-':
        base.turn_r(values['-SLIDER-'])
    
        # base.stop()


window.close()

import PySimpleGUI as sg
import tab_content
import functions
import interfaces
import roslaunch
import os
import rospy
from ROSnode import BASE_CONTROLLER

# Initial setup 
uuid = None
base = None
layout = tab_content.main_layout
window = sg.Window("ROS GUI", layout, size= tab_content.size_window)


def initial_setup(robot_alias):
    # Find workspace "catkin_extras"
    functions.search_ws(folder="catkin_extras", source=True)
    os.environ['ROS_MASTER_URI'] = f'http://{robot_alias}:11311'
    os.environ['ROS_IP'] = interfaces.get_ip_ethernet()  #'169.254.2.172' #'169.254.2.172'
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    rospy.init_node("ROS_GUI")
    base = BASE_CONTROLLER('topic')



while True:
    event, values = window.read()
    if event == sg.WINDOW_CLOSED:
        functions.cleanup_nodes()
        break
    elif event == '-CONNECT-':
        robot_alias = values['-ROBOT_LIST-'][0]
        ip = interfaces.get_robot_ip(robot_alias)
        ip_available = interfaces.ip_is_available(ip)
        if ip_available:
            try:
                initial_setup(robot_alias)
                window['-INFO_CON-'].update(f"Connected to {robot_alias}")
            except:
                window['-INFO_CON-'].update("Error, try again!")
        else:
            window['-INFO_CON-'].update("Error, IP address unreachable!")


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


window.close()

import PySimpleGUI as sg
import tab_content
import functions
import interfaces
import roslaunch
import os
import rospy
import ROSnode

# Initial setup 
global uuid, base
uuid = None
base = None
layout = tab_content.main_layout
window = sg.Window("ROS GUI", layout, size= tab_content.size_window)
setup_completed = False


def initial_setup(robot_alias):
    global uuid, base
    # Find workspace "catkin_extras")
    os.environ['ROS_MASTER_URI'] = f'http://{robot_alias}:11311'
    print("ROS master set")
    os.environ['ROS_IP'] = interfaces.get_ip_ethernet()[0]  #'169.254.2.172' #'169.254.2.172'
    print("ROS ip set")
    functions.search_ws(folder="catkin_extras", source=True)  

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    rospy.init_node("ROS_GUI")
    ROSnode.BASE_CONTROLLER()
    # return uuid, ROSnode.BASE_CONTROLLER('base_controller_topic')



while True:
    event, values = window.read()
    if event == sg.WINDOW_CLOSED:
        functions.cleanup_nodes()
        break
    # Start up
    elif event == '-CONNECT-':
        robot_alias = values['-ROBOT_LIST-'][0]
        ip = interfaces.get_robot_ip(robot_alias)
        ip_available = interfaces.ping_to_ip(ip)
        if ip_available:
            try:
                initial_setup(robot_alias)
                window['-INFO_CON-'].update(f"Connected to {robot_alias}")
                setup_completed = True
            except:
                window['-INFO_CON-'].update("Error, robot is not connected,try again!")
        else:
            window['-INFO_CON-'].update("Error, IP address unreachable!")

    #Launcher
    elif event == '-NAVIGATION-' and setup_completed:
        functions.manage_node('navigation_real', 'nav_pumas', window_event=window[event], uuid=uuid)

    elif event == '-MOVEIT-' and setup_completed:
        functions.manage_node('hsrb_demo_with_controller', 'hsrb_moveit_config', window_event=window[event], uuid=uuid)

    elif event == '-MOVEIT_ARM-' and setup_completed:
        functions.manage_node('arm_test', 'task', window_event=window[event], uuid=uuid)

    elif event == '-LOC-SRV-' and setup_completed:
        functions.manage_node('locations_gui', 'known_locations_tf_server', window_event=window[event], uuid=uuid)

    #Robot control
    elif event == '-FORWARD-' and setup_completed:
        base.forward(values['-SLIDER-'])

    elif event == '-BACKWARD-' and setup_completed:
        base.backward(values['-SLIDER-'])

    elif event == '-LEFT-' and setup_completed:
        base.left(values['-SLIDER-'])

    elif event == '-RIGHT-' and setup_completed:
        base.right(values['-SLIDER-'])

    elif event == '-TURN_L-' and setup_completed:
        base.turn_l(values['-SLIDER-'])
    
    elif event == '-TURN_R-' and setup_completed:
        base.turn_r(values['-SLIDER-'])
    
    # Service callers
    elif event == '-TO_LOCS-' and setup_completed:
        ROSnode.call_known_location_add(values['-LOC_NAME-'])

    elif event == '-TO_KNOWLEDGE-' and setup_completed:
        ROSnode.call_knowledge_place_add()


window.close()

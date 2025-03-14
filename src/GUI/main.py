import PySimpleGUI as sg
import tab_content
import functions
import interfaces
import roslaunch
import os
import rospy
import ROSnode
import file_editor

# Initial setup 
global uuid, base
uuid = None
base = None
layout = tab_content.main_layout
window = sg.Window("ROS GUI", layout, size= tab_content.size_window)
setup_completed = False
active_tab = '-CONTROL_TAB-'


def initial_setup(robot_alias, sim):
    global uuid, base
    # Find workspace "catkin_extras")
    os.environ['ROS_MASTER_URI'] = f'http://{robot_alias}:11311'
    print("ROS master set")
    if not sim:
        os.environ['ROS_IP'] = interfaces.get_ip_ethernet()[0]  #'169.254.2.172' #'169.254.2.172'
        print("ROS ip set")
    functions.search_ws(folder="catkin_extras", source=True)  

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    rospy.init_node("ROS_GUI")
    base = ROSnode.BASE_CONTROLLER("base_controller_topic")
    # return uuid, ROSnode.BASE_CONTROLLER('base_controller_topic')

while True:
    event, values = window.read()
    # print(values)
    if event == sg.WINDOW_CLOSED:
        functions.cleanup_nodes()
        break
    # Start up
    elif event == '-CONNECT-' and values['-ROBOT_LIST-'] != '':
        robot_alias = values['-ROBOT_LIST-'][0]
        sim = values['-SIM-']
        print(sim)
        ip = interfaces.get_robot_ip(robot_alias)
        ip_available = interfaces.ping_to_ip(ip)
        if sim:
            initial_setup(robot_alias, sim)
            window['-INFO_CON-'].update(f"Connected to {robot_alias}")
            setup_completed = True
        elif ip_available:
            try:
                initial_setup(robot_alias, sim)
                window['-INFO_CON-'].update(f"Connected to {robot_alias}")
                setup_completed = True
            except Exception as e:
                window['-INFO_CON-'].update("Error, robot is not connected,try again!")
                print(e)
        else:
            window['-INFO_CON-'].update("Error, IP address unreachable!")
    
    elif event == '-TAB_GROUP-':
        active_tab = values['-TAB_GROUP-']

    #Launcher
    if active_tab == '-LAUNCH_TAB-' and setup_completed:
        if event == '-NAVIGATION-':
            functions.manage_node('navigation_real', 'nav_pumas', 
                                  window_event=window[event], uuid=uuid)

        elif event == '-YOLO-':
            functions.manage_node('object_classification', "object_classification", 
                                  window_event=window[event], uuid=uuid)

        elif event == '-MOVEIT-':
            functions.manage_node('hsrb_demo_with_controller', 'hsrb_moveit_config', 
                                  window_event=window[event], uuid=uuid, launch_args=["use_gui:=false"])

        elif event == '-MOVEIT_ARM-':
            functions.manage_node('arm_test', 'task', 
                                  window_event=window[event], uuid=uuid)
        
        elif event == '-PICKUP_ACT-':
            functions.manage_node("pickup_action", 'task', 
                                  window_event=window[event], uuid=uuid)
            
        elif event == '-START_PICKUP-':
            goal = values['-GOAL_OBJ-']
            print(goal)
            if len(goal) > 0:
                ROSnode.set_pickup_goal(goal)



    #Robot control
    elif active_tab == '-CONTROL_TAB-' and setup_completed:
        if event == '-FORWARD-':
            base.forward(values['-SLIDER-'])

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
        
    # Service callers
    elif active_tab == '-SERVICES_TAB-' and setup_completed:

        if event == '-LOC_SRV-':
            functions.manage_node('locations_gui', 'known_locations_tf_server', 
                                  window_event=window[event], uuid=uuid)
        
        elif event == '-TO_LOCS-':
            ROSnode.call_known_location_add(values['-LOC_NAME-'])

        elif event == '-TO_KNOWLEDGE-':
            ROSnode.call_knowledge_place_add()
    
    elif active_tab == "-STATS_TAB-":
        if event == "-TASK_RUN_BTN-":
            obj_name = values["-PICK_OBJ-"]
            if obj_name:
                file_editor.add_object_run(obj_name)
            
        elif event == "-TASK_RECOG_BTN-":
            obj_name = values["-PICK_OBJ-"]
            if obj_name:
                file_editor.add_object_recog(obj_name)
        elif event == "-TASK_GRASP_BTN-":
            obj_name = values["-PICK_OBJ-"]
            if obj_name:
                file_editor.add_object_grasp(obj_name)
        elif event == "-TASK_STATS_BTN-":
            file_editor.view_stats()


window.close()

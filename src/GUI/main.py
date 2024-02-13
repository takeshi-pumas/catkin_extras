import PySimpleGUI as sg
import tab_content
import functions
import rospy
import roslaunch
import os

# Initial setup 
# Find workspace "catkin_extras"
dir = None
for root, dirs, files in os.walk("/home/"):
    if "catkin_extras" in dirs:
        dir = os.path.join(root, "catkin_extras")
        break

if dir is None:
    print("Error setting bash")
else:
    setup_bash_path = os.path.join(dir, "devel/setup.bash")
    os.system(f"source {setup_bash_path}")
    print(f"Bash setup successfully on {setup_bash_path}!")
# os.environ['ROS_MASTER_URI'] = 'http://hsrb.local_et:11311'
# os.environ['ROS_IP'] = '169.254.2.172'

# roslaunch setup
    global uuid
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

layout = tab_content.main_layout

window = sg.Window("ROS GUI", layout, size= tab_content.size_window)


while True:
    event, values = window.read()
    if event == sg.WINDOW_CLOSED:
        functions.cleanup_nodes()
        break
    elif event == '-NAVIGATION-':
        functions.manage_node('navigation_real', 'nav_pumas', window_event=window[event], uuid=uuid)

    elif event == '-MOVEIT-':
        functions.manage_node('hsrb_demo_with_controller', 'hsrb_moveit_config', window_event=window[event], uuid=uuid)

    elif event == '-MOVEIT_ARM-':
        functions.manage_node('arm_test', 'task', window_event=window[event], uuid=uuid)

window.close()

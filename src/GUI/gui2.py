import PySimpleGUI as sg
import rospy
import roslaunch
import os

# Initial setup 
os.system("source /home/takeshi/catkin_extras/devel/setup.bash")
os.environ['ROS_MASTER_URI'] = 'http://hsrb.local_et:11311'
os.environ['ROS_IP'] = '169.254.2.172'

# roslaunch setup
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

def cleanup_nodes():
    for node in nodes.values():
        node.shutdown()

def set_node(package, launch_file):
    cli_args = [package, f'{launch_file}.launch']
    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    return parent

def manage_node(node_name, package, event):
    if nodes.get(node_name) == None:
        nodes[node_name] = set_node(package, node_name)
        nodes[node_name].start()
        window[event].update(button_color = ('white', 'red'))
    else:
        nodes[node_name].shutdown()
        nodes[node_name] = None
        window[event].update(button_color = ('white', 'green'))


layout = [
    [sg.Text("Press Button 1 to Run Navigation, Press Again to Kill Navigation")],
    [sg.Button("Button 1", key='-NAVIGATION-', button_color=('white', 'green'))],
    [sg.Text("Press Button 2 to Run Object Classification, Press Again to Kill Node")],
    [sg.Button("Button 2", key='-YOLO-', button_color=('white', 'green'))],
    [sg.Text("Press Button 3 to Run Moveit, Press Again to Kill Node")],
    [sg.Button("Button 3", key='-MOVEIT-', button_color=('white', 'green'))],
    [sg.Text("Press Button 4 to Run Moveit test arm, Press Again to Kill Node")],
    [sg.Button("Button 4", key='-MOVEIT_ARM-', button_color=('white', 'green'))],
    [sg.Image(key='-IMAGE-')],
]

window = sg.Window("ROS GUI", layout)

nodes = {}#{'navigation_real':None, 'hsrb_demo_with_controller':None, 'arm_test':None}

while True:
    event, _ = window.read()
    if event == sg.WINDOW_CLOSED:
        cleanup_nodes()
        break
    elif event == '-NAVIGATION-':
        manage_node('navigation_real', 'nav_pumas', event)

    elif event == '-MOVEIT-':
        manage_node('hsrb_demo_with_controller', 'hsrb_moveit_config', event)

    elif event == '-MOVEIT_ARM-':
        manage_node('arm_test', 'task', event)

window.close()

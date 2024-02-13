import PySimpleGUI as sg
from functions import get_available_robots, obtener_direcciones_ip_ethernet

font = ('Arial', 12)
size_launch_btn = (10,2)
size_window = (700, 400)


robot_connection_tab_layout = [[sg.Text('Seleccione un robot para conectar:', font=font)],
              [sg.Combo(values=obtener_direcciones_ip_ethernet(), key='-ROBOT LIST-', font=font, size=(15,1)), 
               sg.Button('Connect', key='-CONNECTION-', font=font), 
               sg.Button('Refresh', key='-REFRESH-', font=font)]]

# launch_tab_layout = [
#     [sg.Text("Press Button 1 to Run Navigation, Press Again to Kill Navigation", font=font)],
#     [sg.Button("Navigation", key='-NAVIGATION-', button_color=('white', 'green'), font=font, size=size_launch_btn)],
#     [sg.Text("Press Button 2 to Run Object Classification, Press Again to Kill Node", font=font)],
#     [sg.Button("Object classification", key='-YOLO-', button_color=('white', 'green'), font=font, size=size_launch_btn)],
#     [sg.Text("Press Button 3 to Run Moveit, Press Again to Kill Node", font=font)],
#     [sg.Button("Moveit", key='-MOVEIT-', button_color=('white', 'green'), font=font, size=size_launch_btn)],
#     [sg.Text("Press Button 4 to Run Moveit test arm, Press Again to Kill Node", font=font)],
#     [sg.Button("Moveit arm test", key='-MOVEIT_ARM-', button_color=('white', 'green'), font=font, size=size_launch_btn)]]

launch_tab_layout = [[sg.Text("")],
    [sg.Text("Press a button to run ROS Nodes, press again to finish them.", key='-Btn info-', font=font, justification='center')],
    [sg.Button("Navigation", key='-NAVIGATION-', button_color=('white', 'green'), font=font, size=size_launch_btn), 
    sg.Button("Object classification", key='-YOLO-', button_color=('white', 'green'), font=font, size=size_launch_btn),
    sg.Button("Moveit", key='-MOVEIT-', button_color=('white', 'green'), font=font, size=size_launch_btn),
    sg.Button("Moveit arm test", key='-MOVEIT_ARM-', button_color=('white', 'green'), font=font, size=size_launch_btn)]
    ]

robot_controller_tab_layout = [[sg.Text("")],
    [sg.Text('', size=(3, 1)), sg.Button('', size=(10, 1)), sg.Text('', size=(3, 1))],
    [sg.Button('', size=(3, 1)), sg.Button('', size=(10, 2)), sg.Button('', size=(3, 1))],
    [sg.Text('', size=(3, 1)), sg.Button('', size=(10, 1)), sg.Text('', size=(3, 1))],
]


main_layout = [robot_connection_tab_layout,
    [sg.TabGroup(
        [[sg.Tab("Robot controllers tab", robot_controller_tab_layout, font=font, element_justification='center'),
            sg.Tab("ROS node launch tab", launch_tab_layout, font=font, element_justification='center')]], size=size_window)]
]
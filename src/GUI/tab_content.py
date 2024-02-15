import PySimpleGUI as sg
from functions import get_available_robots, obtener_direcciones_ip_ethernet, search_ws
from interfaces import get_hosts

font = ('Arial', 12)
size_launch_btn = (10,2)
size_window = (700, 400)

size_column = (300, size_window[1])

ws = search_ws()
assets_dir = f"{ws}/src/GUI/assets/"


robot_connection_tab_layout = [[sg.Text('Seleccione un robot para conectar:', font=font)],
              [sg.Combo(values = get_hosts(), key='-ROBOT_LIST-', font=font, size=(15,1)), 
               sg.Button('Connect', key='-CONNECT-', font=font), 
               sg.Button('Refresh', key='-REFRESH-', font=font),
               sg.Text('Info', key='-INFO_CON-', font=font)]]

launch_tab_layout = [[sg.Text("")],
    [sg.Text("Press a button to run ROS Nodes, press again to finish them.", key='-Btn info-', font=font, justification='center')],
    [sg.Button("Navigation", key='-NAVIGATION-', button_color=('white', 'green'), font=font, size=size_launch_btn), 
    sg.Button("Object classification", key='-YOLO-', button_color=('white', 'green'), font=font, size=size_launch_btn),
    sg.Button("Moveit", key='-MOVEIT-', button_color=('white', 'green'), font=font, size=size_launch_btn),
    sg.Button("Moveit arm test", key='-MOVEIT_ARM-', button_color=('white', 'green'), font=font, size=size_launch_btn)]
    ]

robot_controls_layout = [
    [sg.Text("")],
    [sg.RealtimeButton('', size=(1, 1), key='-TURN_L-',     button_color='white', image_filename=f'{assets_dir}arrow_l.png'), 
     sg.RealtimeButton('', size=(1, 1), key='-FORWARD-',    button_color='white', image_filename=f'{assets_dir}arrow_up.png'), 
     sg.RealtimeButton('', size=(1, 1), key='-TURN_R-',     button_color='white', image_filename=f'{assets_dir}arrow_r.png')],
    [sg.RealtimeButton('', size=(1, 1), key='-LEFT-',       button_color='white', image_filename=f'{assets_dir}arrow_left.png'), 
     sg.RealtimeButton('', size=(1, 1), button_color='white'), 
     sg.RealtimeButton('', size=(1, 1), key='-RIGHT-',      button_color='white', image_filename=f'{assets_dir}arrow_right.png')],
    [sg.Text('', size=(1, 1)), 
     sg.RealtimeButton('', size=(1, 1), key='-BACKWARD-',   button_color='white', image_filename=f'{assets_dir}arrow_down.png'),
     sg.Text('', size=(1, 1))],
     [sg.Text('Velocidad:', font=font),
    sg.Slider(range=(0, 100), default_value=50, orientation='h', size=(20, 13), key='-SLIDER-')],

]

robot_controller_tab_layout = [
    [sg.Column(
        robot_controls_layout, vertical_alignment='top', size=size_column,element_justification='center'),  # Alinea la columna izquierda en la parte superior
     sg.VerticalSeparator(),  # Separador vertical
     sg.Column([
        # Elementos para la parte derecha de la pestaña
        [sg.Text('Contenido de la parte derecha')],
        [sg.Button('Botón 3')],
        [sg.Button('Botón 4')],
        ],size=size_column, vertical_alignment='top')  # Alinea la columna derecha en la parte superior
    ]
]


main_layout = [robot_connection_tab_layout,
    [sg.TabGroup(
        [[sg.Tab("Robot controllers tab", robot_controller_tab_layout, font=font, element_justification='center'),
            sg.Tab("ROS node launch tab", launch_tab_layout, font=font, element_justification='center')]], size=size_window, key='-TAB_GROUP-')]
]

# initial_layout = [robot_connection_tab_layout,
    # [sg.TabGroup(
        # [[sg.Tab("Robot controllers tab", [[sg.Text("Connect to a robot to start", font=font)]], font=font, element_justification='center'),
            # sg.Tab("ROS node launch tab", [[sg.Text("Connect to a robot to start", font=font)]], font=font, element_justification='center')]], size=size_window, key='-TAB_GROUP-')]

# ]
import PySimpleGUI as sg
from functions import get_available_robots, obtener_direcciones_ip_ethernet, search_ws
from interfaces import get_hosts

font = ('Arial', 12)
font2 = ['Arial', 10]
size_launch_btn = (10,2)
size_window = (600, 400)

size_column = (200, size_window[1])
ws = search_ws()
assets_dir = f"{ws}/src/GUI/assets/"


robot_connection_tab_layout = [[sg.Text('Seleccione un robot para conectar:', font=font)],
              [sg.Combo(values = get_hosts(), key='-ROBOT_LIST-', font=font, size=(15,1)), 
               sg.Button('Connect', key='-CONNECT-', font=font), 
               sg.Button('Refresh', key='-REFRESH-', font=font),
               sg.Text('Robot is not connected', key='-INFO_CON-', font=font, text_color='yellow')]]

launch_tab_layout = [[sg.Text("")],
    [sg.Text("Press a button to run ROS Nodes, press again to finish them.", key='-Btn info-', font=font, justification='center')],
    [sg.Button("Navigation", key='-NAVIGATION-', button_color=('white', 'green'), font=font, size=size_launch_btn), 
    sg.Button("Object classification", key='-YOLO-', button_color=('white', 'green'), font=font, size=size_launch_btn),
    sg.Button("Moveit", key='-MOVEIT-', button_color=('white', 'green'), font=font, size=size_launch_btn),
    sg.Button("Moveit arm test", key='-MOVEIT_ARM-', button_color=('white', 'green'), font=font, size=size_launch_btn)]
    ]

hri_tab_layout = [
    [sg.Text("Main services to start tasks", font=font, justification='center')],
    [sg.HorizontalSeparator()],
    [sg.Button("Start location service node", key='-LOC_SRV-', font=font2, button_color=('white', 'green'))], 
    [sg.Text("Location_name:", font=font2), 
     sg.InputText(key='-LOC_NAME-', size = (15, 1)), sg.Button("Add to known locations", key='-TO_LOCS-', font=font2), 
     sg.Button("Add to knowledge (receptionist)", "-TO_KNOWLEDGE-", font=font2)],
     [sg.HorizontalSeparator()]
]

robot_base_controls_layout = [
    [sg.Text("Base controls", font=font2)],
    [sg.RealtimeButton('', size=(1, 1), key='-TURN_L-',     button_color='white', image_filename=f'{assets_dir}arrow_l.png'), 
     sg.RealtimeButton('', size=(1, 1), key='-FORWARD-',    button_color='white', image_filename=f'{assets_dir}arrow_up.png'), 
     sg.RealtimeButton('', size=(1, 1), key='-TURN_R-',     button_color='white', image_filename=f'{assets_dir}arrow_r.png')],
    [sg.RealtimeButton('', size=(1, 1), key='-LEFT-',       button_color='white', image_filename=f'{assets_dir}arrow_left.png'), 
     sg.RealtimeButton('', size=(1, 1), button_color='white'), 
     sg.RealtimeButton('', size=(1, 1), key='-RIGHT-',      button_color='white', image_filename=f'{assets_dir}arrow_right.png')],
    [sg.Text('', size=(1, 1)), 
     sg.RealtimeButton('', size=(1, 1), key='-BACKWARD-',   button_color='white', image_filename=f'{assets_dir}arrow_down.png'),
     sg.Text('', size=(1, 1))],
     [sg.Text('Velocidad:', font=font2),
    sg.Slider(range=(0, 100), default_value=40, orientation='h', size=(13, 13), key='-SLIDER-')],
    # [sg.HorizontalSeparator()],[sg.Text("Head controls", font= font2)]

]

robot_head_controls_layout = [
    [sg.Text("Head controls", font=font2)],
    [sg.Text(''), 
     sg.RealtimeButton('', size=(1, 1), key='-HEAD_UP-',    button_color='white', image_filename=f'{assets_dir}arrow_up.png'), 
     sg.Text('')],
    [sg.RealtimeButton('', size=(1, 1), key='-HEAD_LEFT-',       button_color='white', image_filename=f'{assets_dir}arrow_left.png'), 
     sg.RealtimeButton('', size=(1, 1), button_color='white'), 
     sg.RealtimeButton('', size=(1, 1), key='-HEAD_RIGHT-',      button_color='white', image_filename=f'{assets_dir}arrow_right.png')],
    [sg.Text('', size=(1, 1)), 
     sg.RealtimeButton('', size=(1, 1), key='HEAD_DOWN-',   button_color='white', image_filename=f'{assets_dir}arrow_down.png'),
     sg.Text('', size=(1, 1))],
    #  [sg.Text('Velocidad:', font=font2),
    # sg.Slider(range=(0, 100), default_value=50, orientation='h', size=(20, 13), key='-SLIDER-')],
]


robot_controller_tab_layout = [
    [sg.Column(robot_base_controls_layout, vertical_alignment='top', element_justification='center',
                size=(None, None), expand_x=True),  # Alinea la columna izquierda en la parte superior
     sg.Column(robot_head_controls_layout, vertical_alignment='top', element_justification='center', 
               size=(None, None), expand_x=True),
    #  sg.VerticalSeparator(),  # Separador vertical
     sg.Column([
        # Elementos para la parte derecha de la pestaña
        [sg.Text('Contenido de la parte derecha')],
        [sg.Button('Botón 3')],
        [sg.Button('Botón 4')],
        ], vertical_alignment='top', element_justification='center', size=(None, None), expand_x=True)  # Alinea la columna derecha en la parte superior
    ]
]


main_layout = [robot_connection_tab_layout,
    [sg.TabGroup(
        [[sg.Tab("Robot controllers tab", robot_controller_tab_layout, key='-CONTROL_TAB-', font=font, element_justification='center'),
            sg.Tab("ROS node launch tab", launch_tab_layout, key='-LAUNCH_TAB-', font=font, element_justification='center'),
            sg.Tab("HRI tab", hri_tab_layout, key='-SERVICES_TAB-')]], size=size_window, key='-TAB_GROUP-', enable_events=True)]
]

# initial_layout = [robot_connection_tab_layout,
    # [sg.TabGroup(
        # [[sg.Tab("Robot controllers tab", [[sg.Text("Connect to a robot to start", font=font)]], font=font, element_justification='center'),
            # sg.Tab("ROS node launch tab", [[sg.Text("Connect to a robot to start", font=font)]], font=font, element_justification='center')]], size=size_window, key='-TAB_GROUP-')]

# ]
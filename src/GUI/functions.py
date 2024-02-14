import roslaunch
import os
import netifaces


nodes = {}

def cleanup_nodes():
    if len(nodes.values()) > 0:
        for node in nodes.values():
            node.shutdown()

def set_node(package, launch_file, uuid):
    cli_args = [package, f'{launch_file}.launch']
    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    return parent

def manage_node(node_name, package, window_event, uuid):
    if nodes.get(node_name) == None:
        nodes[node_name] = set_node(package, node_name, uuid)
        nodes[node_name].start()
        window_event.update(button_color = ('white', 'red'))
    else:
        nodes[node_name].shutdown()
        nodes.pop(node_name)
        window_event.update(button_color = ('white', 'green'))

def get_available_robots():
    # Obtener todas las interfaces de red
    interfaces = netifaces.interfaces()

    # Filtrar las direcciones IP de las interfaces Ethernet
    ethernet_addresses = []
    for interface in interfaces:
        if netifaces.AF_INET in netifaces.ifaddresses(interface) and netifaces.AF_LINK in netifaces.ifaddresses(interface):
            addresses = netifaces.ifaddresses(interface)[netifaces.AF_INET]
            for address_info in addresses:
                if 'addr' in address_info:
                    ethernet_addresses.append(address_info['addr'])

    return ethernet_addresses

def obtener_direcciones_ip_ethernet():
    interfaces = netifaces.interfaces()
    direcciones_ip_ethernet = []

    for interface in interfaces:
        # Verificar si la interfaz está activa y es una interfaz Ethernet
        if netifaces.AF_INET in netifaces.ifaddresses(interface) and netifaces.AF_LINK in netifaces.ifaddresses(interface):
            # Excluir direcciones IP de localhost (127.0.0.1) y de interfaces WiFi
            if interface != 'lo' and 'wlan' not in interface:
                direcciones = netifaces.ifaddresses(interface)[netifaces.AF_INET]
                for direccion in direcciones:
                    if 'addr' in direccion:
                        direcciones_ip_ethernet.append(direccion['addr'])

    return direcciones_ip_ethernet

def search_ws(folder = 'catkin_extras', source = False):
    dir = None
    for root, dirs, files in os.walk("/home/"):
        if folder in dirs:
            dir = os.path.join(root, "catkin_extras")
            break
    
    if source:
        if dir is None :
            print("Error setting bash")
        else:
            setup_bash_path = os.path.join(dir, "devel/setup.bash")
            os.system(f"source {setup_bash_path}")
            print(f"Bash setup successfully on {setup_bash_path}!")
    else:
        return dir
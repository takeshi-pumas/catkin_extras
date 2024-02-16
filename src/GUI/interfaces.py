
import netifaces
import socket
import re
import subprocess


def get_ip_ethernet(interfaz = 'enp60s0'):
    ips = []
    try:
        direcciones = netifaces.ifaddresses(interfaz)
        for direccion in direcciones.get(netifaces.AF_INET, []):
            ips.append(direccion['addr'])
    except ValueError:
        pass
    return ips

def is_IPv4(ip):
        # Expresión regular para validar una dirección IPv4
    patron_ipv4 = r'^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$'
    return re.match(patron_ipv4, ip) is not None


def get_hosts(archivo_hosts = '/etc/hosts'):
    hosts = []
    with open(archivo_hosts, 'r') as archivo:
        for linea in archivo:
            # Ignoramos las líneas que comienzan con "#" (comentarios) o están en blanco
            if not linea.strip().startswith("#") and len(linea.strip()) > 0 :
                elementos = linea.split()
                # El primer elemento es la dirección IP, los siguientes son los alias o nombres de host
                if is_IPv4(elementos[0]):
                    hosts.append(elementos[1:])
                # ip = elementos[0]
    return hosts


def get_robot_ip(nombre_host):
    try:
        direccion_ip = socket.gethostbyname(nombre_host)
        return direccion_ip
    except socket.gaierror:
        return None

# def ip_is_available(direccion_ip):
#     try:
#         with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
#             s.settimeout(1)  # Establece un tiempo de espera de 1 segundo para la conexión
#             s.connect((direccion_ip, 80))
#         return True
#     except (socket.timeout, ConnectionRefusedError):
#         return False
    
def ping_to_ip(ip):
    try:
        output = subprocess.check_output(['ping', '-c', '1', ip])
        return True
    except subprocess.CalledProcessError:
        return False

def main():
    print(get_hosts())
    nombre_host = "hsrb.local_et" #"localhost" #"hsrb.local_et"
    direccion_ip = get_robot_ip(nombre_host)
    if direccion_ip:
        print(f"La dirección IP de {nombre_host} es: {direccion_ip}")
    else:
        print(f"No se pudo resolver la dirección IP para {nombre_host}")
    
    if ping_to_ip(direccion_ip):
        print('ip is available')
    else:
        print('ip is not available')
    print(get_ip_ethernet())

if __name__ == "__main__":
    main()




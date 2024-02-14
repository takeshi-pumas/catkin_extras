'''import netifaces

def obtener_ips_interfaz(interfaz):
    ips = []
    try:
        direcciones = netifaces.ifaddresses(interfaz)
        for direccion in direcciones.get(netifaces.AF_INET, []):
            ips.append(direccion['addr'])
    except ValueError:
        pass
    return ips

def main():
    interfaces = ['enp60s0', 'wlp61s0']
    for interfaz in interfaces:
        print(f"IPs para la interfaz {interfaz}:")
        ips = obtener_ips_interfaz(interfaz)
        for ip in ips:
            print(ip)

if __name__ == "__main__":
    main()'''

import socket

def get_robot_ip(nombre_host):
    try:
        direccion_ip = socket.gethostbyname(nombre_host)
        return direccion_ip
    except socket.gaierror:
        return None

def ip_is_available(direccion_ip):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(1)  # Establece un tiempo de espera de 1 segundo para la conexión
            s.connect((direccion_ip, 80))
        return True
    except (socket.timeout, ConnectionRefusedError):
        return False

def main():
    nombre_host = "hsrb.local_et"
    direccion_ip = get_robot_ip(nombre_host)
    if direccion_ip:
        print(f"La dirección IP de {nombre_host} es: {direccion_ip}")
    else:
        print(f"No se pudo resolver la dirección IP para {nombre_host}")
    
    if ip_is_available(direccion_ip):
        print('ip is available')
    else:
        print('ip is not available')

if __name__ == "__main__":
    main()




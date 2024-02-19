import rospy
from geometry_msgs.msg import Twist
from known_locations_tf_server.srv import Locations_server

class BASE_CONTROLLER:
    def __init__(self, topic):
        self._base_vel_pub = rospy.Publisher(
            '/hsrb/command_velocity', Twist, queue_size=1)
        self.MAX_VEL = 0.005

    def forward(self, slider_value):
        vel = slider_value * self.MAX_VEL
        self._publish_msg(velX= vel)
    def backward(self, slider_value):
        vel = slider_value * self.MAX_VEL
        self._publish_msg(velX = -vel)
    def right(self, slider_value):
        vel = slider_value * self.MAX_VEL
        self._publish_msg(velY = -vel)
    def left(self, slider_value):
        vel = slider_value * self.MAX_VEL
        self._publish_msg(velY = vel)
    def turn_l(self, slider_value):
        vel = slider_value * 2 * self.MAX_VEL
        self._publish_msg(velT= -vel)
    def turn_r(self, slider_value):
        vel = slider_value * 2 * self.MAX_VEL
        self._publish_msg(velT = vel)
    def stop(self):
        self._publish_msg()


    def _publish_msg(self, velX = 0, velY = 0, velT= 0):
        twist = Twist()
        twist.linear.x = velX
        twist.linear.y = velY
        twist.angular.z = velT
        self._base_vel_pub.publish(twist)

# class HEAD_CONTROLLER:
#     def __init__(self, topic):
        

# Llamada de servicios
def call_known_location_add(location_name):
    rospy.wait_for_service('/known_location_add', timeout=5)  # Espera a que el servicio esté disponible
    try:
        known_location_add = rospy.ServiceProxy('/known_location_add', Locations_server)  # Crea un proxy para el servicio
        resp = known_location_add(location_name)  # Llama al servicio con el nombre de la ubicación
        return resp.success  # Retorna True si el servicio fue exitoso, False de lo contrario
    except rospy.ServiceException as e:
        print("Error al llamar al servicio: ", e)
        return False

# Función para llamar al servicio de agregar lugar de conocimiento
def call_knowledge_place_add():
    rospy.wait_for_service('/knowledge_place_add', timeout=5)  # Espera a que el servicio esté disponible
    try:
        knowledge_place_add = rospy.ServiceProxy('/knowledge_place_add', Locations_server)  # Crea un proxy para el servicio
        resp = knowledge_place_add()
        return resp.success  # Retorna True si el servicio fue exitoso, False de lo contrario
    except rospy.ServiceException as e:
        print("Error al llamar al servicio: ", e)
        return False




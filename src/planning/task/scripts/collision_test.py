# Importar los módulos necesarios
import rospy
import sys
import moveit_commander
from moveit_msgs.msg import CollisionObject, PlanningScene
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

def add_collision_object():
    # Inicializar el nodo de ROS
    rospy.init_node('add_collision_object_example', anonymous=True)

    # Inicializar MoveIt Commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Esperar a que MoveIt esté listo
    rospy.sleep(1)

    # Crear un objeto de colisión
    collision_object = CollisionObject()
    collision_object.header.frame_id = "base_link"  # Marco de referencia del robot
    collision_object.id = "my_object"  # Identificador único del objeto

    # Definir la geometría del objeto (en este caso, una caja)
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [0.1, 0.1, 0.1]  # Dimensiones de la caja (ancho, alto, profundidad)

    # Definir la posición y orientación del objeto
    pose = Pose()
    pose.position.x = 0.6  # Coordenada x
    pose.position.y = 0.0  # Coordenada y
    pose.position.z = 0.43  # Coordenada z
    pose.orientation.w = 1.0  # Cuaternión de orientación

    # Agregar la geometría y la pose al objeto de colisión
    collision_object.primitives.append(primitive)
    collision_object.primitive_poses.append(pose)

    # Configurar la operación del objeto de colisión para agregarlo a la escena
    planning_scene_diff = PlanningScene()
    planning_scene_diff.is_diff = True
    planning_scene_diff.world.collision_objects.append(collision_object)
    planning_scene_diff.robot_state.is_diff = True

    # Publicar la escena modificada
    planning_scene_publisher = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1)
    rospy.sleep(1)
    planning_scene_publisher.publish(planning_scene_diff)

    rospy.loginfo("Collision object added successfully")

if __name__ == '__main__':
    try:
        add_collision_object()
    except rospy.ROSInterruptException:
        pass

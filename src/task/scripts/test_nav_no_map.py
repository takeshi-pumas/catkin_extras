import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from graph_msgs.msg import GeometryGraph
from navig_msgs.srv import PlanPathRequest, PlanPathResponse, PlanPath

# Callback para obtener el OccupancyGrid
def grid_map_callback(msg):
    global grid_map
    grid_map = msg

def main():
    rospy.init_node('path_planner_client')

    # Suscribirse al tópico para obtener el grid map
    rospy.Subscriber('/navigation/local_grid/local_grid_map', OccupancyGrid, grid_map_callback)

    # Esperar a que el grid map sea recibido
    global grid_map
    grid_map = None
    while grid_map is None:
        rospy.sleep(1)

    # Configurar los mensajes de PoseStamped para el start y goal
    start = PoseStamped()
    start.header.frame_id = "base_link"
    start.pose.position.x = 0.0
    start.pose.position.y = 0.0
    start.pose.position.z = 0.0
    start.pose.orientation.w = 1.0

    goal = PoseStamped()
    goal.header.frame_id = "base_link"
    goal.pose.position.x = 0.5
    goal.pose.position.y = 0.0
    goal.pose.position.z = 0.0
    goal.pose.orientation.w = 1.0

    tolerance = 0.01  # Por ejemplo, 0.5 metros

    # Esperar a que el servicio esté disponible
    rospy.wait_for_service('/path_planner/plan_path_local_grid')

    try:
        # Crear el cliente del servicio
        plan_path_service = rospy.ServiceProxy('/path_planner/plan_path_local_grid', PlanPath)
        
        # Crear la solicitud del servicio
        request = PlanPathRequest()
        request.start = start
        request.goal = goal
        request.tolerance = tolerance
        request.grid_map = grid_map
        request.graph_map = GeometryGraph()

        # Llamar al servicio
        response = plan_path_service(request)
        
        # Manejar la respuesta
        path = response.plan
        rospy.loginfo("Path received: %s", path)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    main()

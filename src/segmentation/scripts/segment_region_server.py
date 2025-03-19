#!/usr/bin/env python3
import rospy
import numpy as np
import ros_numpy
import cv2
import yaml
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
from segmentation.srv import SegmentRegion, SegmentRegionResponse  # Importa el servicio personalizado

class RegionSegmentationServer:
    def __init__(self):
        rospy.init_node("region_segmentation_server")
        rospy.loginfo("Servidor de segmentación de regiones iniciado.")

        # Cargar el archivo YAML con las regiones y el mapa
        yaml_path = rospy.get_param("~map_yaml", "/home/joel/Repositories/catkin_extras/src/navigation_pumas/config_files/prohibition_maps/map_lab_2024/map.yaml")
        rospy.loginfo(f"Cargando YAML del mapa y regiones: {yaml_path}")
        with open(yaml_path, "r") as file:
            map_data = yaml.safe_load(file)

        self.regions_data = map_data["regions"]  # Regiones definidas en el YAML
        self.resolution = map_data["resolution"]  # Resolución del mapa
        self.origin = map_data["origin"]  # Origen del mapa
        rospy.loginfo(f"Resolución del mapa: {self.resolution}, Origen: {self.origin}")

        self.bridge = CvBridge()

        # Configurar transformaciones con TF2
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Crear el servicio
        self.service = rospy.Service("segment_region", SegmentRegion, self.handle_segmentation)
        rospy.loginfo("Servicio 'segment_region' listo para recibir solicitudes.")

    def point_in_region(self, x, y, region_vertices):
        """ Verifica si un punto en metros está dentro de una región. """
        region_poly = np.array(region_vertices, np.float32)
        return cv2.pointPolygonTest(region_poly, (x, y), False) >= 0

    def find_robot_region(self, robot_x, robot_y):
        """ Determina en qué región se encuentra el robot. """
        for region_name, region_data in self.regions_data.items():
            if self.point_in_region(robot_x, robot_y, region_data["vertices"]):
                return region_name
        return None

    def handle_segmentation(self, req):
        """ Maneja la solicitud de segmentación. """
        rospy.loginfo(f"Recibida solicitud de segmentación para la región: {req.region_name}")

        # Verificar que la región exista en el YAML
        if req.region_name not in self.regions_data:
            rospy.logwarn(f"Región '{req.region_name}' no encontrada.")
            return SegmentRegionResponse(success=False, message="Región no encontrada.")

        region_vertices_m = self.regions_data[req.region_name]["vertices"]

        # Convertir la nube de puntos a numpy
        points_data = ros_numpy.numpify(req.pointcloud)

        try:
            # Obtener la transformación de la nube de puntos al sistema del mapa
            trans = self.tfBuffer.lookup_transform('map', 'head_rgbd_sensor_link', rospy.Time(0), rospy.Duration(1.0))
            rospy.loginfo("Transformación de la nube de puntos obtenida.")

            # Aplicar la transformación a la nube de puntos
            cloud_out = do_transform_cloud(req.pointcloud, trans)
            np_corrected = ros_numpy.numpify(cloud_out)
            corrected = np_corrected.reshape(points_data.shape)

            # Extraer la imagen RGB de la nube de puntos corregida
            rgb_image = corrected['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
            rospy.loginfo("Imagen extraída de la nube de puntos.")

            # Extraer coordenadas X, Y (ignoramos Z)
            x_coords = np.nan_to_num(corrected['x'], nan=0.0)
            y_coords = np.nan_to_num(corrected['y'], nan=0.0)

            height, width = rgb_image.shape[:2]
            mask = np.zeros((height, width), dtype=np.uint8)

            # Filtrar los píxeles fuera de la región
            count_inside, count_outside = 0, 0
            for i in range(height):
                for j in range(width):
                    x, y = x_coords[i, j], y_coords[i, j]
                    if self.point_in_region(x, y, region_vertices_m):
                        mask[i, j] = 255  # Mantener el píxel dentro de la región
                        count_inside += 1
                    else:
                        count_outside += 1

            rospy.loginfo(f"Píxeles dentro de la región: {count_inside}, fuera: {count_outside}")

            # Convertir la máscara a un mensaje de ROS
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")

            # Devolver la respuesta con la máscara
            return SegmentRegionResponse(success=True, message="Segmentación completada.", mask=mask_msg)

        except Exception as e:
            rospy.logwarn(f"Error procesando la nube de puntos: {e}")
            return SegmentRegionResponse(success=False, message="Error en el procesamiento.")

if __name__ == "__main__":
    try:
        RegionSegmentationServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

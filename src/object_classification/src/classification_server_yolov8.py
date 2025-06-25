#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
from std_msgs.msg import String
#from object_classification.srv import Classify, ClassifyResponse
from object_classification.srv import Classify_yolo_receptionist, Classify_yolo_receptionistResponse
from object_classification.msg import Floats, Ints
import ros_numpy
import rospkg
import tf2_ros
import tf
import cv2
import numpy as np
import torch

from ultralytics import YOLO 

from utils_srv import RGBD, save_image, write_tf

def callback(req):
    rospy.loginfo(f"Received {len(req.in_.image_msgs)} images")
    res = Classify_yolo_receptionistResponse()
    images = []
    for msg in req.in_.image_msgs:
        image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        images.append(image_rgb)

    for test_img in images:
        results = model(test_img) 
        detections = results[0].boxes
        debug_img = np.copy(test_img)
        num_preds = 0

        points_msg = rospy.wait_for_message(
            '/hsrb/head_rgbd_sensor/depth_registered/rectified_points',
            PointCloud2
        )
        points = ros_numpy.numpify(points_msg)

        for det in detections:
            conf = det.conf[0].item()
            if conf > 0.5:
                xyxy = det.xyxy[0].cpu().numpy().astype(int)
                cls = int(det.cls[0].item())
                pt_min = xyxy[:2].tolist()
                pt_max = xyxy[2:].tolist()

                cc = [
                    np.nanmean(points['x'][pt_min[1]:pt_max[1], pt_min[0]:pt_max[0]]),
                    np.nanmean(points['y'][pt_min[1]:pt_max[1], pt_min[0]:pt_max[0]]),
                    np.nanmean(points['z'][pt_min[1]:pt_max[1], pt_min[0]:pt_max[0]])
                ]

                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = cc
                pose.orientation.w = 1
                res.poses.append(pose)

                if not any(np.isnan(cc)):
                    t = write_tf(cc, (0, 0, 0, 1), model.names[cls], "head_rgbd_sensor_rgb_frame")
                    broadcaster.sendTransform(t)

                debug_img = cv2.rectangle(debug_img, pt_min, pt_max, (0, 255, 0), 2)
                debug_img = cv2.putText(
                    debug_img,
                    model.names[cls],
                    tuple(pt_min),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

                for coord in pt_max:
                    res.pt_max.data.append(coord)
                for coord in pt_min:
                    res.pt_min.data.append(coord)

                res.confidence.data.append(conf)
                string_msg = String()
                string_msg.data = model.names[cls]
                res.names.append(string_msg)

                num_preds += 1
                print(num_preds, pt_min, pt_max, conf, model.names[cls], cc)

        rgb_debug_img = cv2.cvtColor(debug_img, cv2.COLOR_RGB2BGR)
        save_image(rgb_debug_img, name="obj_classif_img")
        res.debug_image.image_msgs.append(bridge.cv2_to_imgmsg(rgb_debug_img, encoding="bgr8"))

    rospy.loginfo(f"### Total detections: {num_preds}")
    return res


def classify_server():
    global bridge, model, broadcaster
    rospy.init_node('classification_server')

    bridge = CvBridge()
    rgbd = RGBD()

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    tf.TransformListener()
    broadcaster = tf2_ros.TransformBroadcaster()

    # Ruta del modelo YOLOv8
    device = "cuda" if torch.cuda.is_available() else "cpu"
    rospack= rospkg.RosPack()
    file_path = rospack.get_path('object_classification')
    ycb_yolo_path=file_path+'/src/weights/yolo_11.pt'

    model=YOLO(ycb_yolo_path)
    rospy.loginfo(f"YOLOv8 model loaded on {device}")
    #model_path = "/home/angel/ANGEL/Angel_YOLO/yolo_11.pt"
    #model = YOLO(model_path)
    rospy.loginfo(f"Loaded YOLOv8 model from {model}")

    service = rospy.Service('classify_yolov8', Classify_yolo_receptionist, callback)
    rospy.loginfo("classification_service (YOLOv8) ready.")
    rospy.spin()


if __name__ == "__main__":
    classify_server()

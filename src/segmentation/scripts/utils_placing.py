#! /usr/bin/env python3
import tf
import cv2
import rospy  
import tf2_ros     
import tf2_geometry_msgs
from segmentation.srv import Segmentation, SegmentationResponse 
from cv_bridge import CvBridge
from object_classification.srv import *
import tf2_ros    
from segmentation.msg import *
import numpy as np
import ros_numpy
import os
import matplotlib.pyplot as plt
import rospkg
import yaml
from sensor_msgs.msg import Image , LaserScan , PointCloud2
from geometry_msgs.msg import TransformStamped, Pose, Vector3Stamped , PointStamped , PoseStamped
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from mpl_toolkits.axes_grid1 import make_axes_locatable
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import seaborn as sns
import pandas as pd
from sklearn.decomposition import PCA
#-----------------------------------------------------------------
global tf_listener, ptcld_lis, broadcaster , bridge , rospack , Pca
import open3d as o3d
rospack = rospkg.RosPack()
#rospy.init_node('plane_segmentation') 
rospy.init_node('placing_finder') 
tfBuffer = tf2_ros.Buffer()
tfBuffer = tf2_ros.Buffer()
listener2 = tf2_ros.TransformListener(tfBuffer)
listener = tf.TransformListener()

broadcaster = tf2_ros.TransformBroadcaster()
tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()

#tf_listener = tf.TransformListener()
#broadcaster= tf.TransformBroadcaster()
#tf_static_broadcaster= tf2_ros.StaticTransformBroadcaster()
#pub = rospy.Publisher('/segmented_images', Image, queue_size=1)
bridge=CvBridge()
Pca=PCA()
#-----------------------------------------------------------------
def points_to_PCA(points):
    df=pd.DataFrame(points)
    df.columns=[['x','y','z']]
    threshold= df['z'].min().values[0]*0.998
    print (threshold)
    rslt_df = df.loc[df[df['z'] > threshold].index]
    points=rslt_df[['x','y','z']].dropna().values
    Pca=PCA(n_components=3)
    Pca.fit(points)
    print('Pca.explained_variance_',Pca.explained_variance_)
    ref=np.eye(3)
    pcas=Pca.components_
    R=[]
    R.append(np.dot(pcas[0],ref))
    R.append(np.dot(pcas[1],ref))
    R.append(np.dot(pcas[2],ref))
    R=np.asarray(R)
    ## HOMOGENEUS
    E_R= np.zeros((4,4))
    E_R[:3,:3]+=R
    E_R[-1,-1]=1
    return     E_R


#-----------------------------------------------------------------
def write_tf(pose, q, child_frame="" , parent_frame='map'):
    #  pose = trans  q = quaternion  , childframe =""
    # format  write the transformstampled message
    t= TransformStamped()
    t.header.stamp = rospy.Time.now()
    #t.header.stamp = rospy.Time(0)
    t.header.frame_id =parent_frame
    t.child_frame_id =  child_frame
    t.transform.translation.x = pose[0]
    t.transform.translation.y = pose[1]
    t.transform.translation.z = pose[2]
    #q = tf.transformations.quaternion_from_euler(eu[0], eu[1], eu[2])
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t
    
#-----------------------------------------------------------------
def read_tf(t):
    # trasnform message to np arrays
    pose=np.asarray((
        t.transform.translation.x,
        t.transform.translation.y,
        t.transform.translation.z
        ))
    quat=np.asarray((
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w
        ))
    
    return pose, quat
#-----------------------------------------------------------------    
def save_auto_plane_region(inlier_points, output_name='auto_plane'):
    import os
    import yaml
    import numpy as np

    mean_inlier = np.mean(inlier_points, axis=0)
    #x_vals = inlier_points[:, 0]
    #y_vals = inlier_points[:, 1]
    #x_min, x_max = float(x_vals.min()), float(x_vals.max())
    #y_min, y_max = float(y_vals.min()), float(y_vals.max())
    z = float(mean_inlier[2])
    def clip_outliers(values, lower=5, upper=95):
        return np.percentile(values, lower), np.percentile(values, upper)

    x_vals = inlier_points[:, 0]
    y_vals = inlier_points[:, 1]
    x_min, x_max = map(float, clip_outliers(x_vals))
    y_min, y_max = map(float, clip_outliers(y_vals))

    quat = [0.0, 0.0, 0.707, 0.707]

    auto_region = {
        output_name: {
            'x_min': x_min,
            'x_max': x_max,
            'y_min': y_min,
            'y_max': y_max,
            'z': z,
            'quat_w': quat[3],
            'quat_x': quat[0],
            'quat_y': quat[1],
            'quat_z': quat[2]
        }
    }

    file_path = rospack.get_path('segmentation') + f'/config_files/{output_name}.yaml'
    with open(file_path, 'w') as outfile:
        yaml.dump(auto_region, outfile, default_flow_style=False)

    print(f"Saved auto region YAML to {file_path}")
    corners = {
        f"{output_name}_corner_1": (x_min, y_min, z),
        f"{output_name}_corner_2": (x_max, y_min, z),
        f"{output_name}_corner_3": (x_min, y_max, z),
        f"{output_name}_corner_4": (x_max, y_max, z),
    }
    for frame_id, position in corners.items():
        tf_msg = write_tf(position, quat, child_frame=frame_id, parent_frame="map")
        broadcaster.sendTransform(tf_msg)
        rospy.loginfo(f"Published TF for {frame_id}")


#-----------------------------------------------------------------    
def read_yaml(yaml_file = '/segmentation_params.yaml'):
    
    file_path = rospack.get_path('segmentation')+'/config_files'  + yaml_file

    with open(file_path, 'r') as file:
        content = yaml.safe_load(file)
    return content



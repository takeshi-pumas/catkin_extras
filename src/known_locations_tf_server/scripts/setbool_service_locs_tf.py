#! /usr/bin/env python3
import rospy  
import tf2_ros    
import numpy as np
from std_msgs.msg import String                                # the main module for ROS-python programs
from known_locations_tf_server.srv import *
import pandas as pd
from geometry_msgs.msg import TransformStamped
def write_tf(pose, q, child_frame , parent_frame='map'):
    t= TransformStamped()
    t.header.stamp = rospy.Time(0)
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
def np_to_str(np_arr):
    str_np=''
    for e in np_arr:
        str_np+= ','+str("%.2f" % e)
    return str_np

def read_tf(t):
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


def callback(req):
    ''' 
    Callback function used by the service server to process
    requests from clients. It returns a TriggerResponse
    '''
    #print(type(req))
    print ()
    resp = Locations_serverResponse()
    #print (resp)
    try:
        trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
        trans.child_frame_id= req.location_name.data
        tf_static_broadcaster.sendTransform(trans)
        print (trans)
        
        #####################################
        trans,quat=read_tf(trans)
        #with  open('known_locations.txt' , 'a') as out:
        
        with  open('/home/takeshi/Codes/known_locations.txt' , 'a') as out:
            out.write (req.location_name.data+np_to_str(trans)+np_to_str(quat)  +'\n' )
        print (trans,quat)
        ####################### 


        resp.success.data= True
        return resp




    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print ( 'No TF FOUND')
        resp.success.data= False
        return resp

    

                 

    

global tfBuffer,tf_static_broadcaster ,known_locs
rospy.init_node('locs_server') 
tfBuffer = tf2_ros.Buffer()
listener2 = tf2_ros.TransformListener(tfBuffer)
broadcaster = tf2_ros.TransformBroadcaster()
tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()


#df=pd.read_csv('known_locations.txt')
df=pd.read_csv('/home/takeshi/Codes/known_locations.txt')##RELATIVIZE PATH?
print (df)
known_locs=df.values
if len(df)!=0:
    for i in range(len(df)):
        trans=df[['x','y','th']].iloc[i].values
        quat=df[['qx','qy','qz','qw']].iloc[i].values
        t=write_tf(trans,quat,df['child_id_frame'].iloc[i])
        print (t,i)
        tf_static_broadcaster.sendTransform(t)
        rospy.sleep(0.3)


rospy.loginfo("known locations detection service available")                    # initialize a ROS node
rospy.Service('/known_location_add', Locations_server, callback         # type, and callback
)

rospy.spin()   

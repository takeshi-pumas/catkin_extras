#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from gazebo_ros import gazebo_interface
from geometry_msgs.msg import Pose, Quaternion
import tf
import subprocess
import os
global save_frame,rgb_mat,_path_model,output_dir,name_output_dir

#####
#Directory that contains all directories of the models
_path_model = '/home/anie/tutorials/catkin_tutorials/src/tmc_wrs_gazebo_world/models/'
############
# Parent directory of the desire path for the data set
output_dir="/home/anie/takeshi/Pictures"
#name of the directory for the data set
name_output_dir="dataset_ycb/"

rgb_mat=[]
save_frame=False

def spawn_object(gazebo_name, name, x, y, z, yaw,roll=0.0 , pitch=0.0):
    global _path_model
    
    _path_xml = _path_model+'MODEL_NAME/model-1_4.sdf'
                
    rospy.loginfo('Spawn: {0}'.format(name))
    initial_pose = Pose()
    initial_pose.position.x = x
    initial_pose.position.y = y
    initial_pose.position.z = z
    
    
    q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    initial_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
    rospy.loginfo('Spawn: {0}'.format(q))

    path_xml = _path_xml.replace('MODEL_NAME', name)

    with open(path_xml, "r") as f:
        model_xml = f.read()

    model_xml = model_xml.replace('PATH_TO_MODEL', _path_model)

    gazebo_interface.spawn_sdf_model_client(gazebo_name, model_xml, rospy.get_namespace(),initial_pose, "", "/gazebo")


def delete_object(name):
    cmd = ['rosservice', 'call', 'gazebo/delete_model',
           '{model_name: ' + str(name) + '}']
    subprocess.call(cmd)



def callback_image(msg):
	global save_frame,rgb_mat
	bridge = CvBridge()
	aux=bridge.imgmsg_to_cv2(msg,msg.encoding)
	rgb_mat=cv2.cvtColor(aux,cv2.COLOR_BGRA2RGB)


def main():
	global rgb_mat, _path_model, output_dir,name_output_dir
	topic='/camera/image_raw'
	rospy.init_node('trainer', anonymous=True)
	subCamera = rospy.Subscriber(topic, Image, callback_image)
	rate = rospy.Rate(20)
	
	

	model_list = os.listdir(_path_model)
	for i in model_list:
		if i[0:3]!='ycb':
			print("eliminando: ",i)
			model_list.remove(i)
	try:
		model_list.remove("wrc_container_b")
		model_list.remove("wrc_long_table")
	except:
		pass

	if output_dir=="":
		dir_curr=os.getcwd()
		base_path = os.path.join(dir_curr, name_output_dir)
	else:
		base_path = os.path.join(output_dir, name_output_dir)

	try:
		os.mkdir(base_path)
	except:
		pass


	##########
	#write item by item for the list
	#model_list=model_list[0:3]
	##########

	state="SMSpaw"
	model=0
	frames=0
	finish=False
	new_model=True
	while not rospy.is_shutdown() and not finish:

		if state=="SMSpaw":
			spawn_object(model_list[model], model_list[model], 0.0,  0.0,  0.0, 0 )
			state="SMWait"

		elif state=="SMWait":
			rospy.sleep(3.)
			state="SMFrames"

		elif state=="SMFrames":
			frames+=1
			if new_model:
				new_model=False
				aux_dir= os.path.join(base_path, model_list[model])
				try:
					os.mkdir(aux_dir)
				except:
					pass

			cv2.imwrite(aux_dir+"/"+str(frames)+".png",rgb_mat)
			if frames>2000:
				state="SMDelete"
				frames=0
				new_model=True
		
		elif state=="SMDelete":
			delete_object(model_list[model])
			model+=1
			if model<len(model_list):
				state="SMSpaw"
			else:
				state="SMFinish"
		elif state=="SMFinish":
			finish=True






if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
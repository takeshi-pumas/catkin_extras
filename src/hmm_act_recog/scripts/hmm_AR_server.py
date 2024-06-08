#!/usr/bin/env python3

from hmm_act_recog.srv import RecognizeOP,RecognizeOPResponse,RecognizeOPRequest
import rospy


from utils_inference import *
from utils_extras import *


def callback(req):

	buf_size=35
	ctrlz=True
	buffer=[]
	cnt_acciones=0
	last_act=-1
	response=RecognizeOPResponse()
	max_inf_it=40
	n_people_max=3

	if req.in_ < 4:
		opWrapper,datum=init_openPose(n_people=1)
	elif req.in_ == 5:
		opWrapper,datum=init_openPose()
	else:
		opWrapper,datum=init_openPose(n_people=n_people_max)

	#----------------
	if req.in_==1:
		response = hmmActionRecognition(datum,opWrapper,response)
    #----------------
	# Para give object
	elif req.in_ == 2:
		response = estimateArmTFforObjectGive(datum,opWrapper,response)
	# Para pointing sin HMM
	elif req.in_ == 3:
		response = getPointingArm(datum,opWrapper,response)
	#----------------
	elif req.in_ == 4:
		response,dataout,image = detectDrinkingPoseWithOP(datum,opWrapper,response,n_people_max = 2)
	
	elif req.in_ == 5:
		response = detectWavingRestaurant(datum,opWrapper,response)
	# Para obtener la imagen y esqueleto 1 vez y trabajar con ella fuera del servicio
	else:
		print("PRUEBA")
		#response = useOpenPoseOnce(datum,opWrapper,response)

	return response
#---------------------------

def recognition_server():
	global tf_listener,rgb,rgbd,bridge,class_names,mA,mB,mPI,opWrapper,datum,cb,h,w
	#---Parte para cargar lo necesario en inferencia con OpenPose y Markov---
	class_names=["wave_R","wave_L","neutral","drink","pointing"]
	mA,mB,mPI=loadModels(class_names)

	cb=np.load(path.join(rospack.get_path("hmm_act_recog"))+"/scripts/codebooks/codebook_LBG_160_s30.npy")
	#opWrapper,datum=init_openPose(n_people=1)
	h=480
	w=640
	# ---

	#--- Parte para cargar lo necesario de ROS

	#rospy.init_node('recognize_action_server')
	rgbd= RGBD()
	rgb= RGB()
	#bridge = CvBridge()
	#tf_man = TF_MANAGER()
	#bridge = CvBridge()
	#tf_listener = tf.TransformListener()
	#broadcaster= tf.TransformBroadcaster()
	#tf_static_broadcaster= tf2.StaticTransformBroadcaster()
	rospy.loginfo("Action recognition service available (NO DOCKER)")                    # initialize a ROS node
	s = rospy.Service('hmm_recognize_act', RecognizeOP, callback) 
	print("Reconition service available")

	rospy.spin()
#---

#========================================
if __name__ == "__main__":
	#rospy.init_node('recognize_action_server')	
    recognition_server()

#!/usr/bin/env python3

from hmm_act_recog.srv import RecogHMMAction, RecogHMMActionResponse,RecogHMMActionRequest
import sys
import rospy
from hmm_act_recog.msg import *
import numpy as np

class_names=["wave_R","wave_L","neutral","drink","pointing"]

if __name__ == "__main__":
	"""
		
		Simple client script to show how the service works 
		using random data 

	"""
	
	print("Testing client ")
	req=RecogHMMActionRequest()

	rospy.wait_for_service('hmm_recognize_act')
	recognize_action = rospy.ServiceProxy('hmm_recognize_act', RecogHMMAction)
	
	# Prueba con dato "random" asimilando una salida de openPose
	# de un video
	buffer_in=np.random.rand(30,25,2)

	print(buffer_in,buffer_in.shape)
	# Es necesario 'aplanar' el array 
	req.sk_buffer=buffer_in.ravel()
	# Asi como mandar la dimension del mismo 
	req.buf_size=np.asarray(buffer_in.shape)

	res=recognize_action(req)

	# Respuesta del servicio, valor del entero asi como la accion
	# correspondiente 
	print(res.i_out,class_names[res.i_out])
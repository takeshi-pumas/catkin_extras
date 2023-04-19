#!/usr/bin/env python3


from hmm_act_recog.srv import RecogHMMAction, RecogHMMActionResponse,RecogHMMActionRequest

import rospy

from utils_hmmAR import *

#---------------------------------------------------------------
def callback(req):
	"""
		Servicio para reconocimiento de acciones usando HMM
			Entradas:
			 Un vector de secuencia de esqueletos, 
			  de tamaño (N*J*C,1) redimensionado de forma:
			      N: el tamaño de la secuencia de la accion
				  J: el numero de joints (puntos del esqueletos) 
					 J -> está pensado en que sea J=30 dado openPose que 
						  retorna 30 joints.
				  C: coordenadas en X,Y -> C=2
			 Otra entrada, las dimensiones N,J,C ya que es necesario 
			  redimensionar el array (N*J*C,1) -> (N,J,C)

			Salida: 
			 Un entero 'i_out' con el valor de la acción inferida 

	"""
	ctrlz=True
	buffer=[]
	response=RecogHMMActionResponse()

	if ctrlz:
	    cb2=centralizaSecuencia(cb,codebook=True)    
	else:
	    cb2=cb
	sk=np.copy(req.sk_buffer.data)
	a,b,c=req.buf_size.data
	
	sk=sk.reshape(int(a),int(b),int(c))

	
	for s in sk:
		buffer.append(create_vk(s,cb2,quitaJ=True,centralized=ctrlz))

	probas=inf_Secuence(np.asarray(buffer),mA,mB,mPI)
                
	
	
	response.i_out=np.argmax(probas[:])
	return response


#---------------------------------------------------------------
def hmm_act_recognition_server():
	global cb,class_names,mA,mB,mPI
	class_names=["wave_R","wave_L","neutral","drink","pointing"]
	mA,mB,mPI=loadModels(class_names)
	cb=np.load("src/hmm_act_recog/scripts/codebooks/codebook_LBG_160_s30.npy")

	rospy.init_node('hmm_act_recognition_server')
	rospy.loginfo("Action recognition HMM service available")                    # initialize a ROS node
	s = rospy.Service('hmm_recognize_act', RecogHMMAction, callback) 
	print("Reconition service available")
	rospy.spin()



#========================================
if __name__ == "__main__":
    hmm_act_recognition_server()
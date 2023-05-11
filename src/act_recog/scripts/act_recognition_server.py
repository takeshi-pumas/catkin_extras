#!/usr/bin/env python3

from act_recog.srv import Recognize,RecognizeResponse,RecognizeRequest
from act_recog.msg import Floats 
import rospy


from utils_inference import *
from utils_extras import *

def callback(req):

	flo=Floats()
	buf_size=35
	ctrlz=True
	buffer=[]
	cnt_acciones=0
	last_act=-1
	response=RecognizeResponse()
	max_inf_it=40
	#----------------
	if req.in_==1:
		print("Opcion 1\n\tObteniendo imagenes...")
		dataout=np.zeros((25,2))
		if ctrlz:
		    cb2=centralizaSecuencia(cb,codebook=True)    
		else:
		    cb2=cb
		while True:
			# <<<<<<<<<<<<<<<< obtengo imagen >>>>>>>>>>>>>>>>>>>>>
			im=rgbd.get_image()
			imgOut=np.copy(im)
			#im=cv2.cvtColor(im, cv2.COLOR_BGR2RGB )

			#    Obtengo esqueleto
			#       Obtengo simbolo y append
			# <<<<<<<<<<<< obtengo esqueleto >>>>>>>>>>>>>>>>>>>>>>>>>
			datum.cvInputData = im
			opWrapper.emplaceAndPop(op.VectorDatum([datum]))
			if datum.poseKeypoints is not None:
			    dataout=np.copy(datum.poseKeypoints[0,:,:2])
			    # <<<< si detecto esqueleto, obtengo su simbolo y lo guardo >>>>>>>>
			    symbol=create_vk(dataout,cb2,quitaJ=True,centralized=ctrlz)
			    buffer.append(symbol)
			else:
			    symbol=0




			if len(buffer)==buf_size+1:
			    buffer.pop(0)
			#<<<<< si el buffer tiene un cierto tamaño, empieza la inferencia >>>>>>
			if len(buffer)==buf_size:
			    probas=inf_Secuence(np.asarray(buffer),mA,mB,mPI)
			    #print("Accion inferida: {}".format(class_names[np.argmax(probas[:])]))
			    # <<<<<<< empieza a contar repeticiones de acciones inferidas >>>>>>
			    if last_act==-1:
			        last_act=np.argmax(probas[:])

			    else:
			        if np.argmax(probas[:])==last_act:
			            cnt_acciones+=1
			        else:
			            cnt_acciones=0
			            last_act=np.argmax(probas[:])

			im=draw_skeleton(dataout,h,w,im,cnt_person=0,bkground=True)
			im=draw_skeleton(cb[symbol],h,w,im,cnt_person=0,bkground=True,centroid=True)
			if last_act==-1:
			    cv2.putText(img=im, text="buffer size:"+str(len(buffer))+", symbol det:"+str(symbol)+" reps:"+str(cnt_acciones), 
			    org=(5, 20),fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=(35, 255, 148),thickness=2)
			else:
			    cv2.putText(img=im, text="buffer size:"+str(len(buffer))+", symbol det:"+str(symbol)+" reps:"+str(cnt_acciones)+" act:"+str(class_names[last_act]), 
			    org=(5, 20),fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=(35, 255, 148),thickness=2)
			if req.visual!=0:
			    cv2.imshow("Imagen RGB",im)
			    cv2.waitKey(10)

			#<<<<<<<<<<< si la inferencia se repite un determinado numero de veces
			#            termina el ciclo y regresa variables y demas >>>>>>>>>>>>

			if cnt_acciones==max_inf_it and last_act==4:
			    print("Accion 4 detectada: {}".format(class_names[last_act]))
			    img_msg=bridge.cv2_to_imgmsg(imgOut) # change to rgbd in robot
			    sk_msg=bridge.cv2_to_imgmsg(dataout)
			    print("Recibiendo imagen rgbd...")
			    frameC,dataPC=get_coordinates()
			    print("publicando...")
			    frameC=draw_skeleton(dataout,h,w,frameC,cnt_person=0,bkground=True)  
			    #if req.visual!=0:  
			    #    cv2.imshow("IMAGEN RGBD",frameC)
			    #    cv2.waitKey(400)
			    pub_points(dataPC,dataout,skPub=1)
			    print("tfs publicadas")

			    break


			# Si es drink o neutral, sigue reconociendo hasta obtener otra accion
			elif cnt_acciones==max_inf_it and (last_act==2 or last_act==3):
			    print("Accion detectada: neutral o drink, se sigue detectando... ")
			    cnt_acciones=0

			# Si es waving, similar a pointing pero solo publica la cara
			elif cnt_acciones==max_inf_it:
			    print("Accion detectada: {}".format(class_names[last_act]))
			    print("Recibiendo imagen rgbd...")
			    frameC,dataPC=get_coordinates()
			    print("publicando...")
			    pub_points(dataPC,dataout,skPub=0)
			    print("tf publicada")
			    img_msg=bridge.cv2_to_imgmsg(imgOut) # change to rgbd in robot
			    sk_msg=bridge.cv2_to_imgmsg(dataout)
			    
			    break
		if req.visual!=0:
		    cv2.destroyAllWindows()
		    cv2.waitKey(1)


		if len(response.im_out.image_msgs)==0:
		    response.im_out.image_msgs.append(img_msg)
		else:
		    response.im_out.image_msgs[0]=img_msg

		
		response.i_out=np.argmax(probas[:])

		return response
    #----------------
	# Para give object
	elif req.in_==2:
		print("Opcion 2, estimar brazo para dar objeto".format(req.in_))
		cnt=0
		while True:

			im=rgbd.get_image()
			dataout=np.zeros((25,2))
			datum.cvInputData = im
			opWrapper.emplaceAndPop(op.VectorDatum([datum]))
			if datum.poseKeypoints is not None:
			    cnt+=1
			    dataout=np.copy(datum.poseKeypoints[0,:,:2])
			    im=draw_skeleton(dataout,h,w,im,cnt_person=0,bkground=True)
			    cv2.putText(img=im, text="Contador: "+str(cnt),org=(5, 20),fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
			                fontScale=0.6, color=(35, 255, 148),thickness=2)

			    if req.visual!=0:
			        cv2.imshow("Imagen RGB",im)
			        cv2.waitKey(10)
			    if cnt==50:
			        
			        print("Obteniendo rgbd...")
			        frameC,dataPC=get_coordinates()
			        print("esqueleto encontrado")

			        mano,codo=detect_pointing_arm(dataout,dataPC)
			        tf_man.pub_static_tf(pos=codo,point_name='CODO',ref='head_rgbd_sensor_link')
			        tf_man.pub_static_tf(pos=mano,point_name='MANO',ref='head_rgbd_sensor_link')
			        #print("cambiando referencia")
			        tf_man.change_ref_frame_tf(point_name='CODO',new_frame='map')
			        tf_man.change_ref_frame_tf(point_name='MANO',new_frame='map')

			        print("tf publicada")

			        response.i_out=1
			        break
		print("Cerrando CV2")
		if req.visual!=0:
			print("SI")    
			cv2.destroyAllWindows()
			cv2.waitKey(1)

		img_msg=bridge.cv2_to_imgmsg(im)
		img_msg2=bridge.cv2_to_imgmsg(dataout)
		if len(response.im_out.image_msgs)==0:
		    response.im_out.image_msgs.append(img_msg)
		else:
		    response.im_out.image_msgs[0]=img_msg

		
		return response
    #----------------
	# Para pointing sin HMM
	elif req.in_==3:
		max_inf_it=40
		print("Opcion {}, estimar brazo que esta apuntando".format(req.in_))
		cnt=0
		while True:

			im=rgb.get_image()
			#im=cv2.cvtColor(im, cv2.COLOR_BGR2RGB)

			dataPC=rgbd.get_points()
			dataout=np.zeros((25,2))
			skeletons_xyz=np.zeros((25,3))
			datum.cvInputData = im
			opWrapper.emplaceAndPop(op.VectorDatum([datum]))
			if datum.poseKeypoints is not None:
			    cnt+=1
			    dataout=np.copy(datum.poseKeypoints[0,:,:2])
			    
			    if req.visual!=0:
			    	im=draw_skeleton(dataout,h,w,im,cnt_person=0,bkground=True)
			    	cv2.putText(img=im, text="Contador: "+str(cnt),org=(5, 20),fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
			                fontScale=0.6, color=(35, 255, 148),thickness=2)
			    	cv2.imshow("Imagen RGB",im)
			    	cv2.waitKey(10)
			    #print(cnt)
			    if cnt==max_inf_it:
			        flg=0
			        print("Obteniendo rgbd...")

			        # Para evitar lecturas nan y no retorne coordenadas nan

			        while flg!=1:
				        frameC,dataPC=get_coordinates()
				        im=cv2.cvtColor(frameC, cv2.COLOR_BGR2RGB)
				        print("esqueleto encontrado")
				        # Calculo una ultima vez con openpose y la imagen que se obtiene de pointcloud
				        datum.cvInputData = im
				        opWrapper.emplaceAndPop(op.VectorDatum([datum]))
				        if datum.poseKeypoints is not None:
				        	dataout=np.copy(datum.poseKeypoints[0,:,:2])
				    	# Y lo guardo

				        im_t=draw_skeleton(dataout,h,w,im,cnt_person=0,bkground=True)

				        #cv2.imshow("Imagen y sk para extrapola y TF ",im_t)
				        #cv2.waitKey(0)

				        mano,codo,f=detect_pointing_arm(dataout,dataPC)
				        print("MANO CODO",mano,codo)
				        if f!=-1:
					        tf_man.pub_static_tf(pos=codo,point_name='CODO',ref='head_rgbd_sensor_link')
					        tf_man.pub_static_tf(pos=mano,point_name='MANO',ref='head_rgbd_sensor_link')
					        #print("cambiando referencia")
					        tf_man.change_ref_frame_tf(point_name='CODO',new_frame='map')
					        tf_man.change_ref_frame_tf(point_name='MANO',new_frame='map')
					        rospy.sleep(0.8)

					        manoM,_ = tf_man.getTF(target_frame='MANO',ref_frame='map')
        					codoM,_ = tf_man.getTF(target_frame='CODO',ref_frame='map')
				        	ob_xyz = get_extrapolation(manoM,codoM)
				        	flg=1
				        else:
				        	print("HAY UNA NAN. Recalcula PC...")
			        rospy.sleep(0.8)

			        response.i_out=f
			        break

		#if req.visual!=0:
		#	cv2.destroyAllWindows()
		#	cv2.waitKey(1)
		#print("DATA SK")
		#print(dataout)
		img_msg=bridge.cv2_to_imgmsg(im)
		
		flo.data=ob_xyz

		response.d_xyz=flo
		img_msg2=bridge.cv2_to_imgmsg(dataout)
		if len(response.im_out.image_msgs)==0:
		    response.im_out.image_msgs.append(img_msg2)
		else:
		    response.im_out.image_msgs[0]=img_msg

		#print(response)
		return response

	#----------------
	# Para obtener la imagen y esqueleto 1 vez y trabajar con ella fuera del servicio
	else:
		print("Opcion {}, una sola imagen con openpose".format(req.in_))


		im=rgbd.get_image()
		dataout=np.zeros((25,2))
		datum.cvInputData = im
		opWrapper.emplaceAndPop(op.VectorDatum([datum]))
		if datum.poseKeypoints is not None:
		    dataout=np.copy(datum.poseKeypoints[0,:,:2])
		    print("Obteniendo rgbd...")
		    frameC,dataPC=get_coordinates()
		    print("esqueleto encontrado")
		    #pub_points(dataPC,dataout,skPub=1)
		    print("tf publicada")
		    if dataout[0,0]!=0 and dataout[0,1]!=0:
		        response.i_out=1
		    else:
		        print("Se detecto esqueleto pero no la cara de la persona")
		        response.i_out=-1
		else:
		    print("No se encontro esqueleto/persona")
		    response.i_out=-1

		img_msg=bridge.cv2_to_imgmsg(im)
		img_msg2=bridge.cv2_to_imgmsg(dataout)
		if len(response.im_out.image_msgs)==0:
		    response.im_out.image_msgs.append(img_msg)
		else:
		    response.im_out.image_msgs[0]=img_msg

		
		return response    


#global tf_man
def recognition_server():
	global tf_listener,rgb,rgbd, bridge,class_names,mA,mB,mPI,opWrapper,datum,cb,h,w
	#---Parte para cargar lo necesario en inferencia con OpenPose y Markov---
	class_names=["wave_R","wave_L","neutral","drink","pointing"]
	mA,mB,mPI=loadModels(class_names)

	cb=np.load(path.join(rospack.get_path("act_recog"))+"/scripts/codebooks/codebook_LBG_160_s30.npy")
	opWrapper,datum=init_openPose(n_people=1)
	h=480
	w=640
	# ---

	#--- Parte para cargar lo necesario de ROS

	#rospy.init_node('recognize_action_server')
	rgbd= RGBD()
	rgb= RGB()
	#bridge = CvBridge()
	#tf_listener = tf.TransformListener()
	#broadcaster= tf.TransformBroadcaster()
	#tf_static_broadcaster= tf2.StaticTransformBroadcaster()
	rospy.loginfo("Action recognition service available")                    # initialize a ROS node
	s = rospy.Service('recognize_act', Recognize, callback) 
	print("Reconition service available")

	rospy.spin()
#---

#========================================
if __name__ == "__main__":
    recognition_server()

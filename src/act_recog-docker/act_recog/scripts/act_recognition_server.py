#!/usr/bin/env python3

from act_recog.srv import Recognize,RecognizeResponse,RecognizeRequest
from act_recog.msg import Floats 
import rospy


from utils_inference import *
from utils_extras import *


"""
	Servicio usando docker para utilizar OpenPose sin ocuparse
	de si la computadora lo tiene instalado, con el objetivo de 
	inferir distintas acciones
	Este servicio no usa HMM para la deteccion de esqueletos,
	quedó en otro servicio con distinto nombre

"""


#========================================
def callback(req):

	flo=Floats()
	buf_size=35
	ctrlz=True
	buffer=[]
	cnt_acciones=0
	last_act=-1
	response=RecognizeResponse()
	n_people_max=-1
	if req.in_ <= 3:
		opWrapper,datum=init_openPose(n_people = 1)
	
	elif req.in_ == 5:
		opWrapper,datum=init_openPose()
		
	else:
		opWrapper,datum=init_openPose(n_people=n_people_max)

	if req.in_==1: 	# Para utilizar otra camara, por ejemplo usb_cam

		print("Opcion 1: USB_CAM y no accion inferida, solo pose con Openpose")
		data = rospy.wait_for_message("/usb_cam/image_raw",Image,timeout=5) #
		cv2_img = bridge.imgmsg_to_cv2(data)
		image=np.copy(cv2_img)
		h,w,_=image.shape
		

		datum.cvInputData = image
		opWrapper.emplaceAndPop(op.VectorDatum([datum]))
		if datum.poseKeypoints is not None:
			dataout=np.copy(datum.poseKeypoints[0,:,:2])
			response.i_out=1
			image=draw_skeleton(dataout,h,w,image,cnt_person=0,bkground=True)
		else:
			print("NO SK DETECTADO")
		img_msg=bridge.cv2_to_imgmsg(image)
		if len(response.im_out.image_msgs)==0:
			response.im_out.image_msgs.append(img_msg)
		else:
			response.im_out.image_msgs[0]=img_msg

		if req.visual==1:
			cv2.imshow("RES",image)
			cv2.waitKey(0)

	# Para give object 
	elif req.in_==2:
		print("Opcion 2, estimar brazo para dar objeto".format(req.in_))
		cnt=0

		while True:

			im=rgbd.get_image()
			#################### USE WITH USB_CAM #############################
			#data = rospy.wait_for_message("/usb_cam/image_raw",Image,timeout=5) #
			#cv2_img = bridge.imgmsg_to_cv2(data)
			#im=np.copy(cv2_img)
			#######################################################################
			h,w,_=im.shape
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
		if len(response.im_out.image_msgs)==0:
			response.im_out.image_msgs.append(img_msg)
		else:
			response.im_out.image_msgs[0]=img_msg
		
		#return response
	
	elif req.in_==3:
		max_inf_it=30
		print("Opcion {}, estimar brazo que esta apuntando".format(req.in_))
		cnt=0
		while True:

			im=rgb.get_image()
			#im=cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
			h,w,_=im.shape
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
							dataout=np.copy(datum.poseKeypoints[0,:,:2])# Y lo guardo
						
						im_t=draw_skeleton(dataout,h,w,im,cnt_person=0,bkground=True)
						
						mano,codo,f=detect_pointing_arm(dataout,dataPC)
						print("MANO CODO",mano,codo)
						if f!=-1:
							tf_man.pub_static_tf(pos=codo,point_name='CODO',ref='head_rgbd_sensor_link')
							tf_man.pub_static_tf(pos=mano,point_name='MANO',ref='head_rgbd_sensor_link')#print("cambiando referencia")
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
			
		#return response	
	
	# para bebida (mal)
	elif req.in_==4:
		conteo_sin_bebida=np.zeros(n_people_max)
		max_drink_cnt=10
		m_no_p=10
		c_nor=10
		cnt_normal=0
		no_person=0
		flg_out=False
		while True:
			image,dataPC=get_coordinates()#print(image.shape)
			h,w,_=image.shape
			datum.cvInputData = image
			opWrapper.emplaceAndPop(op.VectorDatum([datum]))#print("Body keypoints: \n" + str(datum.poseKeypoints))
			if datum.poseKeypoints is not None:
				no_person=0
				dataout=np.copy(datum.poseKeypoints[:,:,:2])
				order=np.argsort(np.argsort(dataout[:,0,0]))
				for i in range(dataout.shape[0]):
					image=draw_skeleton(dataout[i],h,w,image,bkground=True)
					draw_text_bkgn(image,text="Person:"+str(i),pos=(int(dataout[i,0,0]), int(dataout[i,0,1])-40),
		                   font_scale=1.3,text_color=(255, 255, 32))
					draw_rect_sk(dataout[i],image)
					if detect_drinking(dataout[i]):
						conteo_sin_bebida[i]=0
						cnt_normal+=1
						draw_text_bkgn(image,text="Person "+str(i)+": ",pos=(int(dataout[i,0,0]), int(dataout[i,0,1])-40),
		                           font_scale=1.3,text_color=(32, 255, 255))
						draw_text_bkgn(image,text="Con bebida",pos=(int(dataout[i,0,0]), int(dataout[i,0,1])-20),
		                           font_scale=1.3,text_color=(32, 255, 255))
					else:
						cnt_normal=0
						draw_text_bkgn(image,text="Person "+str(i)+":",pos=(int(dataout[i,0,0]), int(dataout[i,0,1])-40),
		                           font_scale=1.3,text_color=(32, 255, 255))
						draw_text_bkgn(image,text="Sin bebida",pos=(int(dataout[i,0,0]), int(dataout[i,0,1])-20),
		                           font_scale=1.3,text_color=(32, 255, 255))
						conteo_sin_bebida[i]+=1
			else:
				no_person+=1
			
			print(conteo_sin_bebida)# -----------------------
			if req.visual!=0:
				cv2.imshow("RES",image)
				cv2.waitKey(10)# --------------------------
			if no_person==m_no_p:
				response.i_out=3
				break
			if cnt_normal==c_nor:
				print("TODOS CON BEBIDA DURANTE UN TIEMPO RAZONABLE")
				response.i_out=2
				break
			else:
				for c in range(n_people_max):
					if conteo_sin_bebida[c]==max_drink_cnt:
						print("\n\nNO TIENE BEBIDA LA PERSONA :{}, PROCEDO A OFRECER UNA\n".format(c))
						flg_out=True
						head,f=return_xyz_sk(dataout,c,dataPC)
						if f!=-1:
							print(head)
							response.i_out=1
							tf_man.pub_static_tf(pos=head,point_name='head_xyz',ref='head_rgbd_sensor_link')
							
							tf_man.change_ref_frame_tf(point_name='head_xyz',new_frame='map')
							rospy.sleep(0.8)
							ob_xyz,_ = tf_man.getTF(target_frame='head_xyz',ref_frame='map')
							print(ob_xyz)#flo.data=ob_xyz#response.d_xyz=flo
							break
						else: 
							response.i_out=0
							print("DATOS NAN, no se retorna datos")
				if flg_out:
					break
		#---------------------------------
		if req.visual!=0:
			cv2.destroyAllWindows()
		#--------------------------------

		#---------------

	#RESTAURANTE
	elif req.in_ ==5 :
		#print("OPCION 5 PARA RESTAURANT")
		counting=0
		while True:

			points_msg=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2,timeout=5)
			points_data = ros_numpy.numpify(points_msg)
			image,maskedImage = removeBackground(points_msg,distance = 10)
			h,w,_=image.shape
			#dataout=np.zeros((25,2))
			datum.cvInputData = maskedImage
			opWrapper.emplaceAndPop(op.VectorDatum([datum]))
			if datum.poseKeypoints is None:
				
				print("No se encontro esqueleto/persona")
				response.i_out=-1
				img_msg=bridge.cv2_to_imgmsg(maskedImage)
				if len(response.im_out.image_msgs)==0:
					response.im_out.image_msgs.append(img_msg)
				else:
					response.im_out.image_msgs[0]=img_msg
			
			else:
				#print("datum shape",datum.poseKeypoints.shape)
				dataout=np.copy(datum.poseKeypoints[:,:,:2])
				for i in range(dataout.shape[0]):
					maskedImage=draw_skeleton(dataout[i],h,w,maskedImage,bkground=True)
					draw_text_bkgn(maskedImage,text="Person "+str(i)+": ",pos=(int(dataout[i,0,0]), int(dataout[i,0,1])-40),
									font_scale=1.3,text_color=(32, 255, 255))
					
				img_msg=bridge.cv2_to_imgmsg(maskedImage)
				if len(response.im_out.image_msgs)==0:
					response.im_out.image_msgs.append(img_msg)
				else:
					response.im_out.image_msgs[0]=img_msg
				sk_idx = detectWaving(dataout,maskedImage,points_msg)
				if sk_idx == -1:
					print("NO PERSON WAVING")
					response.i_out=-1
					counting = 0
				else:
					counting += 1

			if counting == 15:
				break

		#----	
		head_mean = np.concatenate((dataout[sk_idx,0:1,:],dataout[sk_idx,15:19,:]),axis=0)
		head_mean = np.sum(head_mean,axis=0)/np. count_nonzero(head_mean,axis=0)[0] 
		

		head_xyz =[points_data['x'][int(head_mean[1]), int(head_mean[0])],
					points_data['y'][int(head_mean[1]), int(head_mean[0])],
					points_data['z'][int(head_mean[1]), int(head_mean[0])]]
		print("HEAD XYZ of waving person", head_xyz)

		if head_xyz[0] is not None:
			print("PUBLICANDO....")
			tf_man.pub_static_tf(pos=head_xyz,point_name='person_waving',ref='head_rgbd_sensor_link')
			rospy.sleep(0.8)
			print("CAMBIANDO REF")
			tf_man.change_ref_frame_tf(point_name='person_waving',new_frame='map')
			rospy.sleep(0.8)
			response.i_out = 1
		else:
			print("No se pudo publicar")
			response.i_out = 2
	
		img_msg2=bridge.cv2_to_imgmsg(dataout)
		response.im_out.image_msgs.append(img_msg2)

	

		return response


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
			print("esqueleto encontrado")#pub_points(dataPC,dataout,skPub=1)
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
	if req.visual!=0:
		cv2.destroyAllWindows()
	return response    
#========================================

def recognition_server():
	global tf_listener,rgb,rgbd,bridge,class_names,opWrapper,datum,h,w
	#opWrapper,datum=init_openPose(n_people=1)

	# ---

	#rospy.init_node('recognize_action_server')
	rgbd= RGBD()
	rgb= RGB()

	rospy.loginfo("Action recognition service available")                    # initialize a ROS node
	s = rospy.Service('recognize_act', Recognize, callback) 
	print("Reconition service available")

	rospy.spin()
#---

#========================================
if __name__ == "__main__":
	#rospy.init_node('recognize_action_server')	
    recognition_server()

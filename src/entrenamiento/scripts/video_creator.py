#!/usr/bin/env python
import cv2
import os
# Parent directory of the desire path for the vido directory
path_dataset="/home/takeshi/Pictures/dataset_ycb"

def fun_sort(element):
	return int(element[:-4])

if path_dataset=="":
	path_dataset=os.getcwd()

model_list = os.listdir(path_dataset)


video_path = os.path.join(path_dataset, "videos")
try:
	os.mkdir(video_path)
except:
	pass

frame_array=[]

try:
	model_list.remove("videos")
except:
	pass


for model in model_list:
	aux_name=model.split("_")
	aux_name[0]="_"
	aux_name="".join(aux_name)
	video_name=video_path+"/green"+aux_name+"_"
	print(video_name)
	video = cv2.VideoWriter(video_name+'.mp4',cv2.VideoWriter_fourcc(*'mp4v'), 30, (1280, 720))
	frame_array=[]
	model_dir=os.path.join(path_dataset, model)
	frames=os.listdir(model_dir)
	frames.sort(key=fun_sort)
	print("creating video: "+model)
	for i in frames:
		img=cv2.imread(model_dir+"/"+i)
		frame_array.append(img)
		#print(img.shape)

	for i in range(len(frame_array)):
		video.write(frame_array[i])
	video.release()

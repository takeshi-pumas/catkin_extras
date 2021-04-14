import cv2
import os

path_dataset="/home/takeshi/prubas_noobies/catkin_tutorials/dataset_ycb"

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
	video_name=video_path+"/green_"+model
	video = cv2.VideoWriter(video_name+'.mp4',cv2.VideoWriter_fourcc(*'mp4v'), 30, (1280, 720))
	frame_array=[]
	model_dir=os.path.join(path_dataset, model)
	frames=os.listdir(model_dir)
	frames.sort(key=fun_sort)
	print("creating video: "+model)
	for i in frames:
		img=cv2.imread(model_dir+"/"+i)
		img=cv2.cvtColor(img,cv2.COLOR_BGRA2RGB)
		frame_array.append(img)
		#print(img.shape)

	for i in range(len(frame_array)):
		video.write(frame_array[i])
	video.release()

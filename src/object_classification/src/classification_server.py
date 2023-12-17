#!/usr/bin/env python3
from cv_bridge import CvBridge
from object_classification.srv import Classify,ClassifyResponse, ClassifyRequest
from object_classification.msg import Floats  , Ints
import cv2
import numpy as np
####################
import torch
import torch.backends.cudnn as cudnn
from yolov5.models.experimental import attempt_load
from yolov5.utils.general import *
from yolov5.utils.torch_utils import select_device #select_device, load_classifier, time_sync
######################################################################################
from utils_srv import *
#############################################################################
def callback(req):
    
    print ('got ',len(req.in_.image_msgs),'images')    
    res=ClassifyResponse()
    images=[]
    for i in range(len(req.in_.image_msgs)):
        images.append(bridge.imgmsg_to_cv2(req.in_.image_msgs[i]))
    for test_img in images:
        img = torch.from_numpy(test_img).to(device) # RGB IMAGE TENSOR (TORCH)
        img = img / 255.0                              #NORMALIZE
        img=img.unsqueeze(0)                        # ADD DIMENSION FOR TENSOR ( BATCH)
        img=torch.moveaxis(img,3,1)                  #Channel order for YOLO
        pred = model(img, augment=False)[0]
        pred = non_max_suppression(pred)  # IOU 
        debug_img=np.copy(test_img)
        num_preds=0
        for  det in pred:
            for *xyxy, conf, cls in (det):# Model Result is bounding box  confidence  and class
                if conf.cpu().tolist() > 0.5:
                    num_preds+=1
                    pt_min=[int(xyxy[0].cpu().tolist()),int(xyxy[1].cpu().tolist())]
                    pt_max=[int(xyxy[2].cpu().tolist()),int(xyxy[3].cpu().tolist())]
                    debug_img=cv2.rectangle(debug_img ,pt_min,pt_max,  (0, 255, 0), 2   )
                    debug_img= cv2.putText(
                                debug_img ,model.names[int(cls.cpu().tolist())],
                                pt_min,
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 255, 0),
                                2
                                )
                    
                    print (num_preds,pt_min, pt_max,conf.cpu().tolist(),model.names[int(cls.cpu().tolist())])
                    for coord in pt_min:    res.pt_min.data.append(coord)
                    for coord in pt_max:    res.pt_max.data.append(coord)
                    res.confidence.data.append(conf)                    
        np.save('debug.npy',debug_img)  # IMAGE WITH BOUNDING BOX
        print(f'### number of detections -> {num_preds}')
    res.debug_image.image_msgs.append(bridge.cv2_to_imgmsg(debug_img))
    return res 

def classify_server():
    global listener,rgbd, bridge , model , device
    rospy.init_node('classification_server')
    rgbd= RGBD()
    bridge = CvBridge()
    listener = tf.TransformListener()
    broadcaster= tf.TransformBroadcaster()
    tf_static_broadcaster= tf2_ros.StaticTransformBroadcaster()
    device = select_device('')
    model=attempt_load('/home/roboworks/catkin_extras/src/yolov5_ros/scripts/yolov5/ycb.pt',device)
    rospy.loginfo("calssification_ YOLOV5 service available")                    # initialize a ROS node
    s = rospy.Service('classify', Classify, callback) 
    print("Classification service available")
    rospy.spin()
if __name__ == "__main__":
    classify_server()

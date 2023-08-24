#!/usr/bin/env python3

import argparse
import numpy as np
from pathlib import Path
import sys
#sys.path.append("/home/roboworks/hsr_ws/src/yolo-ros-docker")
sys.path.append("/catkin_ws/src/yolo-ros-docker")

import cv2
from cv_bridge import CvBridge
import rospy

import torch
import torch.backends.cudnn as cudnn

from yolov5.models.experimental import attempt_load
from yolov5.utils.dataloaders import LoadStreams, LoadImages
from yolov5.utils.general import *
#from yolov5.utils.general import check_img_size, check_requirements, check_imshow, colorstr, non_max_suppression, \
    #apply_classifier, scale_boxes, scale_segments, xyxy2xywh, strip_optimizer, set_logging, increment_path#, save_one_box
from yolov5.utils.augmentations import letterbox
from yolov5.utils.plots import colors, Annotator
from yolov5.utils.torch_utils import * #select_device, load_classifier, time_sync

from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from yolov5_ros.msg import RecognitionObject, RecognitionObjectArray

bridge = CvBridge()

class Detector:
    def __init__(self,weights='',  # model.pt path(s)
                 imgsz=640,  # inference size (pixels)
                 conf_thres=0.5,  # confidence threshold
                 iou_thres=0.45,  # NMS IOU threshold
                 max_det=1000,  # maximum detections per image
                 device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
                 view_img=True,  # show results default False
                 classes=None,  # filter by class: --class 0, or --class 0 2 3
                 agnostic_nms=False,  # class-agnostic NMS
                 augment=False,  # augmented inference
                 visualize=False,  # visualize features
                 update=False,  # update all models
                 line_thickness=3,  # bounding box thickness (pixels)
                 hide_labels=False,  # hide labels
                 hide_conf=False,  # hide confidences
                 half=False,  # use FP16 half-precision inference
                 topic='image'):
        self.device = select_device(device)
        self.augment = augment
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.classes = classes
        self.agnostic_nms = agnostic_nms
        self.max_det = max_det
        self.view_img = view_img
        self.hide_labels = hide_labels
        self.hide_conf = hide_conf
        self.line_thickness = line_thickness
        self.update = update
        self.sub_topic_name = topic
        
        self.half = half
        self.half &= self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        w = weights[0] if isinstance(weights, list) else weights
        self.classify, suffix = False, Path(w).suffix.lower()

        self.model = attempt_load(weights, device=self.device)  # load FP32 model
        self.stride = int(self.model.stride.max())  # model stride
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names  # get class names
        if self.half:
            self.model.half()  # to FP16
        if self.classify:  # second-stage classifier
            self.modelc = load_classifier(name='resnet50', n=2)  # initialize
            self.modelc.load_state_dict(torch.load('resnet50.pt', device=self.device)['model']).to(self.device).eval()

        self.imgsz = check_img_size(imgsz, s=self.stride)  # check image size
        self.imgsz = [640,480]
        # Dataloader
        cudnn.benchmark = True  # set True to speed up constant image size inference

        self.sub = rospy.Subscriber(self.sub_topic_name, Image, self.image_callback)
        # self.pub = rospy.Publisher("yolov5_obj", PoseStamped, queue_size=1)
        self.pub = rospy.Publisher("yolov5_obj", RecognitionObjectArray, queue_size=1)

    @torch.no_grad()
    def process_img(self, cv_image):
        result_str = ""
        #img0 = cv2.flip(cv_image, 1)  # flip left-right
        img0 = cv_image
        # Padded resize
        img = letterbox(img0, self.imgsz, stride=self.stride, auto=True)[0]
        
        # Convert
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = np.ascontiguousarray(img)

        # Run inference
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, *self.imgsz).to(self.device).type_as(next(self.model.parameters())))  # run once

        #print(len(img), len(img0)) #1frame 1frame
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32

        img = img / 255.0  # 0 - 255 to 0.0 - 1.0
        if len(img.shape) == 3:
            img = img[None]  # expand for batch dim

        pred = self.model(img, augment=self.augment, visualize=False)[0]

        # NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)

        # Second-stage classifier (optional)
        if self.classify:
            pred = apply_classifier(pred, self.modelc, img, img0)

        # Process predictions
        recog_obj_arr = RecognitionObjectArray()
        for i, det in enumerate(pred):  # detections per image
            s, im0, = f'{i}: ', img0.copy()
            s += '%gx%g ' % img.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)  # integer class
                    label = None if self.hide_labels else (self.names[c] if self.hide_conf else f'{self.names[c]} {conf:.2f}')
                    annotator.box_label(xyxy, label, color=colors(c, True))
                    result_str += '{},{},{},{},{},{} '.format(int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3]), self.names[c], float(conf))
                    recog_obj = RecognitionObject()
                    recog_obj.x_min = int(xyxy[0])
                    recog_obj.y_min = int(xyxy[1])
                    recog_obj.x_max = int(xyxy[2])
                    recog_obj.y_max = int(xyxy[3])
                    recog_obj.confidence = float(conf)
                    recog_obj.class_name = self.names[c]
                    recog_obj_arr.array.append(recog_obj)

            # Print time (inference + NMS)
            #print(f'{s}Done. ({t2 - t1:.3f}s)')
            #print("a",result_str)
            # Stream results
            if self.view_img:
                cv2.imshow("detectwindow", im0)
                cv2.waitKey(1)

        if self.update:
            strip_optimizer(weights)  # update model (to fix SourceChangeWarning)

        #print(f'Done. ({time.time() - t0:.3f}s)')

        return result_str, recog_obj_arr
        
    def image_callback(self, msg):
        cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        pub_msg, tmp_msg = self.process_img(cv_image)
        tmp_msg.header = msg.header
        self.pub.publish(tmp_msg)
        # p_msg = PoseStamped()
        # p_msg.header.stamp = rospy.Time.now()
        # p_msg.header.frame_id = str(pub_msg)
        # self.pub.publish(p_msg)
        
    # TODO:
    # def detect_srv(self, msg):
    #     self.process_img(msg)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default='yolov5s.pt', help='model.pt path(s)')
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.5, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='show results')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--visualize', action='store_true', help='visualize features')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--line-thickness', default=1, type=int, help='bounding box thickness (pixels)')
    parser.add_argument('--hide-labels', default=False, action='store_true', help='hide labels')
    parser.add_argument('--hide-conf', default=False, action='store_true', help='hide confidences')
    parser.add_argument('--half', action='store_true', help='use FP16 half-precision inference')
    parser.add_argument('--topic', type=str, default='/camera/rgb/image_rect_color', help='ros topic')
    
    opt = parser.parse_args()
    detector = Detector(**vars(opt))
    
    rospy.init_node("yolov5_detector")
    rospy.spin()

#!/usr/bin/env python3
from transformers import VisionEncoderDecoderModel, ViTImageProcessor, AutoTokenizer, pipeline, AutoProcessor, AutoImageProcessor, LlavaForConditionalGeneration
import torch
import PIL.Image

import os, sys
import cv2 as cv

import rospy, rospkg, yaml
from rospkg import RosPack
rp = RosPack()
from visualization_msgs.msg import *

# import numpy as np

from std_msgs.msg import String, Int32, Float32, Int8

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import termios

torch.cuda.empty_cache()
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model_id="llava-hf/llava-1.5-7b-hf"
model = LlavaForConditionalGeneration.from_pretrained(
    model_id, 
    torch_dtype=torch.float16, 
    low_cpu_mem_usage=True,
    load_in_4bit=True
    )

tokenizer = AutoTokenizer.from_pretrained(model_id)
image_processor = AutoImageProcessor.from_pretrained(model_id)
processor = AutoProcessor.from_pretrained(model_id)
mm_pipeline = pipeline("image-to-text", model, image_processor = image_processor, tokenizer = tokenizer)

def image_callback(data):
    global cvbridge, img_name, img_num, exit_capture, photo_snap, prompt
    if img_name == None:
        return

    if exit_capture:
        return

    im = cvbridge.compressed_imgmsg_to_cv2(data)
    # Show the world through Palpy's eyes and allow the user to save snapshots or exit
    # cv.namedWindow(img_name, cv.WINDOW_AUTOSIZE)
    # cv.imshow(img_name, im)
    # cv.waitKey(1)
    # print("Image should be shown")
    if (photo_snap):
        image_to_save = './receptionist/' + img_name + '.jpg'
        cv.imwrite(image_to_save, im)
        print('Captured ' + image_to_save[18:])
        # img_list.append(im)
        photo_snap = False
        # img_num += 1
    k = cv.waitKey(27)
    if k == 27:         # wait for ESC key to exit
        cv.destroyAllWindows()
        exit_capture = True
    elif k == ord('x'):         # wait for x key to exit
        cv.destroyAllWindows()
        exit_capture = True



# def describe_person(image):
#     model_id="llava-hf/llava-1.5-7b-hf"
#     mm_pipeline = pipeline("image-to-text",model_id)
#     prompt = "<image>\nUSER: Please list gender, mood, posture, and clothing of the person. \nASSISTANT:"
#     # image = '/home/kasia/Downloads/person.jpg'
#     output = mm_pipeline(image, prompt=prompt, generate_kwargs={"max_new_tokens": 200})
#     rest, n = output[0]["generated_text"].rsplit('ASSISTANT:', 2)
#     print(n)
#     f = open("person_description.txt", "w")
#     f.write(n)
#     f.close()

def describe_person(image, name, prompt):
    global processor
    print("Describe person")
    image = PIL.Image.open(image)
    inputs = processor(prompt, image, return_tensors='pt').to(0, torch.float16)
    # print(inputs)
    # output = model.generate(**inputs, generate_kwargs={"max_new_tokens": 200}, do_sample=False)
    output = model.generate(**inputs, max_new_tokens=200, do_sample=False)
    text = (processor.decode(output[0][2:], skip_special_tokens=True))
    # output = mm_pipeline(inputs, prompt=prompt, generate_kwargs={"max_new_tokens": 200})
    # print(output)
    rest, n = text.rsplit('ASSISTANT: ', 2)
    print(n)
    f_name = img_name + "_person_description.txt"
    f = open(f_name, "w")
    f.write(n)
    f.close()

def command_callback(data):
    global go_mark, img_name, prompt
    print(data)
    
    if "guest_1" in data.data:
      img_name = "guest_1"
      prompt = "<image>\nUSER: Please list gender, hair color, clothing, if they are wearing glasses of the person. \nASSISTANT:"
    elif "shoes" in data.data:
      img_name = "shoes"
      prompt = "<image>\nUSER: Is the person in the photo wearing shoes? \nASSISTANT:"
    elif "drink" in data.data:
      img_name = "drink"
      prompt = "<image>\nUSER: Is the person in the photo holding a drink? \nASSISTANT:"
    elif "gpsr" in data.data:
      img_name = "thing"
      prompt = data.data
      prompt = prompt.replace("Go gpsr ", "")
      print(prompt)
      # prompt = "<image>\nUSER: Is the person in the photo holding a drink? \nASSISTANT:"
    
    else:
      if not os.path.exists('/home/robocanes/hsr_robocanes/guest_1_person_description.txt'):
        return
      with open("/home/robocanes/hsr_robocanes/guest_1_person_description.txt", "r") as file:
        description = file.read()
        if "chair_2" in data.data:
          img_name = "chair_2"
          prompt = "<image>\nUSER: Does person in the photo match the appearance from the following description:" + description +" Answer yes or no. \nASSISTANT:"
        elif "sofa_1" in data.data:
          img_name = "sofa_1"
          prompt = "<image>\nUSER: Does person in the photo match the appearance from the following description:" + description +" Answer yes or no. \nASSISTANT:"
        elif "sofa_2" in data.data:
          img_name = "sofa_2"
          prompt = "<image>\nUSER: Does person in the photo match the appearance from the following description:" + description +" Answer yes or no. \nASSISTANT:"
        elif "chair_1" in data.data:
          img_name = "chair_1"
          prompt = "<image>\nUSER: Does person in the photo match the appearance from the following description:" + description +" Answer yes or no. \nASSISTANT:"



    if "Go" in data.data:
        go_mark = True
    


if __name__ == '__main__':

  global cvbridge, img_name, img_num, img_list, exit_capture, go_mark, prompt

  # Variables that control taking pic
  go_mark = False
  photo_snap = False
  exit_capture = True

  cvbridge = CvBridge()
  img_name = None

  if os.path.exists('./receptionist'):
      os.rmdir('./receptionist')
  
  os.makedirs('./receptionist')

  # Init node
  rospy.init_node('person_description_receptionist', anonymous=True)

  # Subscribe to Palpy's camera
  rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_raw/compressed", CompressedImage, image_callback)
  rospy.Subscriber("take_photo", String, command_callback)

  while True:
    # Wait for message to take photo
    while not go_mark:
      photo_snap = True

    print(img_name)

    img_path = os.getcwd() + '/receptionist/' + img_name + '.jpg'
    print(img_path)

    while not os.path.exists(img_path):
      exit_capture = False
      go_mark = False
        # img_num = 1
        # img_list = []
    exit_capture = False

    describe_person(img_path, img_name, prompt)

    img_name = None
    exit_capture = True

    print("Done")


 
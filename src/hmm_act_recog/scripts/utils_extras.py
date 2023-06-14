#!/usr/bin/env python3
import numpy as np
import rospy
import ros_numpy
import tf
import tf2_ros as tf2
from sensor_msgs.msg import Image ,PointCloud2

import cv2

#---------------------------------------------------
# Para escribir texto en imagen con fondo negro
def draw_text_bkgn(img, text,
          font=cv2.FONT_HERSHEY_PLAIN,
          pos=(0, 0),
          font_scale=1,
          font_thickness=2,
          text_color=(0, 255, 0),
          text_color_bg=(0, 0, 0)
          ):

    x, y = pos
    text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
    text_w, text_h = text_size
    cv2.rectangle(img, pos, (x + text_w, y + text_h), text_color_bg, -1)
    cv2.putText(img, text, (x, y + text_h + int(font_scale) - 1), font, font_scale, text_color, font_thickness)

    return text_size



def draw_skeleton(joints,hh,wh,im,cnt_person=0,norm=False,bkground=False,centroid=False):

    """
    conections 15 (of 25) joints:
        0-1   <-> centro cabeza         - cuello-torso
        0-11  <-> centro cabeza         - ojo D ?
        0-12  <-> centro cabeza         - ojo I ?
        1-2   <-> cuello-torso          - hombro D
        1-5   <-> cuello-torso          - hombro I
        1-8   <-> cuello-torso          - tronco-cadera ombligo
        2-3   <-> hombro D              - codo D
        3-4   <-> codo D                - muñeca D
        5-6   <-> hombro I              - codo I
        6-7   <-> codo I                - muñeca I
        8-9   <-> tronco-cadera ombligo - tronco-cadera D 
        8-10  <-> tronco-cadera ombligo -tronco-cadera I 
        11-13 <-> ojo D ?               - oreja D
        12-14 <-> ojo I ?               - oreja I

    conections 14 (of 18) joints:

        0-1  <-> centro cabeza  - cuello-torso
        0-10 <-> centro cabeza  - ojo D
        0-11 <-> centro cabeza  - ojo I
        1-2  <-> cuello-torso   - hombro D
        1-5  <-> cuello-torso   - hombro I
        1-8  <-> cuello-torso   - tronco-cadera D
        1-9  <-> cuello-torso   - tronco-cadera I
        2-3  <-> hombro D       - codo D
        3-4  <-> codo D         - muneca D
        5-6  <-> hombro I       - codo I
        6-7  <-> codo I         - muneca I
        10-12<-> ojo D          - oreja D
        11-13<-> ojo I          - oreja I
 

    """
    h=1
    w=1
    lineThick=2
    circleSize=3

    if norm:
        h=hh
        w=wh

    if bkground:
        bkgn=im.astype(np.uint8)
    else:
        bkgn=np.zeros((hh,wh,3),np.uint8)
    
    if centroid:
        lnCnt=int(joints.shape[0]/2)
        frame=np.zeros((lnCnt,2))
        frame[:,0]=joints[:lnCnt]
        frame[:,1]=joints[lnCnt:]
        if frame.shape[0]==15:
            conections=[[0,1],[0,11],[0,12],[1,2],[1,5],[1,8],
                [2,3],[3,4],[5,6],[6,7],[8,9],[8,10],
                [11,13],[12,14]]
        else:
            conections=[[0,1],[0,14],[0,15],[1,2],[1,5],[1,8],
                [1,11],[2,3],[3,4],[5,6],[6,7],[8,9],
                [9,10],[11,12],[12,13],[14,16],[15,17]]

        
        for conect in conections:
            if frame[conect[0]][0]!=0 and frame[conect[1]][1]!=0:
                bkgn=cv2.line(bkgn,(int(frame[conect[0]][0]*h),int(frame[conect[0]][1]*w)),(int(frame[conect[1]][0]*h),int(frame[conect[1]][1]*w)),(0,255,255),lineThick)
        for i in range(frame.shape[0]):
                    if frame[i][0]!=0.0 and frame[i][1]!=0.0:
                        bkgn=cv2.circle(bkgn,(int(frame[i][0]*h),int(frame[i][1]*w)),circleSize,(190,152,253),-1)

        return bkgn

    else:

        if joints.shape[0]==15:
            conections=[[0,1,0],[0,11,1],[0,12,2],[1,2,3],[1,5,4],[1,8,5],
                        [2,3,6],[3,4,7],[5,6,8],[6,7,9],[8,9,10],[8,10,11],
                        [11,13,12],[12,14,13]]
            # 0-1|0-11|0-12|1-2|1-5|1-8
            # 2-3|3-4|5-6|6-7|8-9|8-10 
            # 11-13|12-14
            colors=[(41,23,255),(99,1,249),(251,10,255),(10,75,255),(41,243,186),(10,10,255),   
                    (25,136,253),(40,203,253),(0,218,143),(0,218,116),(78,218,0),(253,183,31),   
                    (248,8,207),(248,8,76)]            

        elif joints.shape[0]==18:
            conections=[[0,1,0],[0,14,1],[0,15,2],[1,2,3],[1,5,4],[1,8,5],
                        [1,11,5],[2,3,6],[3,4,7],[5,6,8],[6,7,9],[8,9,10],
                        [9,10,11],[11,12,12],[12,13,13],[14,16,1],[15,17,1]]
            colors=[(253,45,31),(253,31,104),(253,31,184),(3,14,250),(15,104,252),(72,219,0),
                    (192,219,0),(18,170,255),(50,220,255),(50,255,152),(50,255,82),(113,219,0),
                    (167,251,77),(219,171,0),(219,113,0),(253,31,159),(159,31,253)]

        elif joints.shape[0]==25:
            conections=[[0,1,0],[0,15,1],[0,16,2],[1,2,3],[1,5,4],[1,8,5],
                        [2,3,6],[3,4,7],[5,6,8],[6,7,9],[8,9,10],[8,12,11],
                        [9,10,12],[10,11,13],[11,22,13],[11,24,13],[12,13,14],[13,14,15],
                        [14,19,15],[14,21,15],[15,17,16],[16,18,17],[19,20,15],[22,23,13]]
            # 0-1|0-15|0-16|1-2|1-5|1-8
            # 2-3|3-4|5-6|6-7|8-9|8-12 
            # 9-10|<10-11|11-22|11-24|22-23>|12-13|<13-14|14-19|14-21|19-20>|15-17|16-18
            colors=[(41,23,255),(99,1,249),(251,10,255),(10,75,255),(41,243,186),(10,10,255),
                    (25,136,253),(40,203,253),(0,218,143),(0,218,116),(78,218,0),(253,183,31),
                    (148,241,4),(239,255,1),(253,145,31),(253,80,31),(248,8,207),(248,8,76)]

        else:  #18 to less joints
            conections=[[0,1,0],[0,10,1],[0,11,2],[1,2,3],[1,5,4],[1,8,5],
                        [1,9,5],[2,3,6],[3,4,7],[5,6,8],[6,7,9],
                        [10,12,1],[11,13,1]]

            colors=[(253,45,31),(253,31,104),(253,31,184),(3,14,250),(15,104,252),(72,219,0),
                    (192,219,0),(18,170,255),(50,220,255),(50,255,152),(50,255,82),(113,219,0),
                    (167,251,77),(219,171,0),(219,113,0),(253,31,159),(159,31,253)]

        for i in range(joints.shape[0]):
            if joints[i][0]!=0.0 and joints[i][1]!=0.0:
                bkgn=cv2.circle(bkgn,(int(joints[i][0]*h),int(joints[i][1]*w)),circleSize,(255,255,255),-1)

        for conect in conections:
            if joints[conect[0]][0]!=0 and joints[conect[1]][1]!=0:
                bkgn=cv2.line(bkgn,(int(joints[conect[0]][0]*h),int(joints[conect[0]][1]*w)),(int(joints[conect[1]][0]*h),int(joints[conect[1]][1]*w)),colors[conect[2]],lineThick)
        
        draw_text_bkgn(bkgn,text="Person:"+str(cnt_person),pos=(int(joints[0,0]), int(joints[0,1])-40),
                   font_scale=1.3,text_color=(255, 255, 32))
        return bkgn
#---------------------------------------------------
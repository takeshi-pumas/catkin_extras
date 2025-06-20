#!/usr/bin/env python

import rospy
import os

def receive_audio():
    rospy.init_node('audio_receiver', anonymous=True)
    rate = rospy.Rate(10)  # Frecuencia de publicación
    
    while not rospy.is_shutdown():
        # Configura la tubería de GStreamer para recibir el flujo de audio
        cmd = "gst-launch-1.0 udpsrc port=5004 ! 'application/x-rtp, media=(string)audio, clock-rate=(int)48000, encoding-name=(string)OPUS, payload=(int)96' ! rtpopusdepay ! opusdec ! audioconvert ! autoaudiosink"
        os.system(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        receive_audio()
    except rospy.ROSInterruptException:
        pass


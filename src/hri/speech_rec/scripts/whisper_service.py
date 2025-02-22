#!/usr/bin/env python

import io
from pydub import AudioSegment
import speech_recognition as sr
import whisper
import tempfile
import os
import argparse

#ROS
import std_msgs.msg
from ros_whisper_vosk.srv import GetSpeech, GetFakeSpeech

import rospy
import rospkg

temp_dir = tempfile.mkdtemp()
save_path = os.path.join(temp_dir, "temp.wav")

parser = argparse.ArgumentParser()
parser.add_argument("--model", default="base", type=str, help="Model to use: tiny, base, small, medium, large")
parser.add_argument("--english", default=False, action='store_true', help="Whether to use English model")
parser.add_argument("--verbose", default=False, action='store_true', help="Whether to print verbose output")
parser.add_argument("--energy", default="300", type=int, help="Energy level for mic to detect")
parser.add_argument("--dynamic_energy", default=False, action='store_true', help="Flag to enable dynamic engergy")
parser.add_argument("--pause", default=0.8, type=float, help="Pause time before entry ends")

args = parser.parse_args(rospy.myargv()[1:])

print("------------------------------------------------")
print(args)
print("------------------------------------------------")

#there are no english models for large
model = args.model
if args.model != "large" and args.english:
    model = args.model + ".en"
print(model)
audio_model = whisper.load_model(model)    

#load the speech recognizer and set the initial energy threshold and pause threshold
r = sr.Recognizer()
r.energy_threshold = args.energy
r.pause_threshold = args.pause
r.dynamic_energy_threshold = args.dynamic_energy

#Start ROS Publishers
pub_final = rospy.Publisher('speech_recognition/final_result',std_msgs.msg.String, queue_size=10)


#Callbacks
def callbackWhisperService(req):

    predicted_text = ""
    with sr.Microphone(sample_rate=16000) as source:
        print("Say something!")

        #get and save audio to wav file
        audio = r.listen(source)
        data = io.BytesIO(audio.get_wav_data())
        audio_clip = AudioSegment.from_file(data)
        audio_clip.export(save_path, format="wav")

        if args.english:
            result = audio_model.transcribe(save_path,language='english')
        else:
            result = audio_model.transcribe(save_path)

        if not args.verbose:
            predicted_text = result["text"].lstrip()
            print("You said: " + predicted_text)
        else:
            print(result)

    pub_final.publish(predicted_text)
    return predicted_text

def callbackFakeWhisperService(req):

    predicted_text = req.text
    predicted_text = predicted_text.lstrip()
    print("You said: " + predicted_text)
    
    pub_final.publish(predicted_text)
    return predicted_text

#Main
def main():
    rospy.init_node('whisper_service')
  
    #Start ROS Services
    rospy.Service("speech_recognition/whisper_service", GetSpeech, callbackWhisperService)
    rospy.Service("speech_recognition/fake_whisper_service", GetFakeSpeech, callbackFakeWhisperService)
    
    rospy.loginfo("Whisper Service Initialized")
    
    #Infinite loop
    rospy.spin()

if __name__ == "__main__":
    main()

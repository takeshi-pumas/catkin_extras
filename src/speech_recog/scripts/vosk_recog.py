#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import json
import queue
import vosk
import sounddevice as sd
import rospy
import rospkg
from ros_whisper_vosk.msg import speech_recognition
from std_msgs.msg import String, Bool
from ros_whisper_vosk.srv import SetGrammarVosk, SetGrammarVoskResponse

class VoskSR:
    def __init__(self):
        self.init_ros()
        self.init_audio()
        self.init_model()
        self.rec = None
        self.format_grammar = None
        self.listening_enabled = True

    
    def init_ros(self):
        rospy.init_node('vosk', anonymous=False)
        rospy.on_shutdown(self.cleanup)
        self.pub_vosk = rospy.Publisher('speech_recognition/vosk_result', speech_recognition, queue_size=10)
        self.pub_final = rospy.Publisher('speech_recognition/final_result', String, queue_size=10)
        self.pub_partial = rospy.Publisher('speech_recognition/partial_result', String, queue_size=10)
        rospy.Service('set_grammar_vosk', SetGrammarVosk, self.set_grammar)
        rospy.Subscriber('speech_recognition/enable', Bool, self.toggle_listening)
        self.msg = speech_recognition()
        self.q = queue.Queue()
    
    def init_audio(self):
        self.input_dev_num = rospy.get_param('~input_device_index', sd.query_hostapis()[0]['default_input_device'])
        if self.input_dev_num == -1:
            rospy.logfatal('No input device found')
            raise ValueError('No input device found, device number == -1')
        device_info = sd.query_devices(self.input_dev_num, 'input')
        self.samplerate = int(device_info['default_samplerate'])
        rospy.set_param('vosk/sample_rate', self.samplerate)
    
    def init_model(self):
        package_path = rospkg.RosPack().get_path('ros_whisper_vosk')
        model_dir = os.path.join(package_path, 'models', 'vosk-model-small-en-us-0.15')
        if not os.path.exists(model_dir):
            rospy.logfatal("No model found in %s", model_dir)
            raise FileNotFoundError("Model directory not found")
        self.model = vosk.Model(model_dir)
    
    def cleanup(self):
        rospy.logwarn("Shutting down VOSK speech recognition node...")
    
    def stream_callback(self, indata, frames, time, status):
        if status:
            print(status, file=sys.stderr)
        if self.listening_enabled:
            self.q.put(bytes(indata))
    
    def set_grammar(self, data):
        if not self.rec:
            return SetGrammarVoskResponse(False)
        self.rec.Reset()
        grammar_list = data.grammar + ["[unk]"]
        self.format_grammar = json.dumps(grammar_list)
        return SetGrammarVoskResponse(True)
    
    def toggle_listening(self, msg):
        self.listening_enabled = msg.data
        rospy.loginfo("Listening enabled: %s", self.listening_enabled)
        if not self.listening_enabled:
            with self.q.mutex:
                self.q.queue.clear()
            if self.rec:
                self.rec.Reset()
    
    def speech_recognize(self):
        try:
            with sd.RawInputStream(samplerate=self.samplerate, blocksize=16000, device=self.input_dev_num, dtype='int16', channels=1, callback=self.stream_callback):
                rospy.logdebug('Started recording')
                self.rec = vosk.KaldiRecognizer(self.model, self.samplerate)
                print("Vosk is ready to listen!")
                while not rospy.is_shutdown():

                    if not self.listening_enabled:
                        rospy.sleep(0.1)
                        continue

                    data = self.q.get()
                    if self.rec.AcceptWaveform(data):
                        result = json.loads(self.rec.FinalResult())
                        if len(result.get("text", "")) > 2:
                            self.publish_result(result["text"], True)
                        self.rec.Reset()
                    else:
                        partial_result = json.loads(self.rec.PartialResult()).get("partial", "")
                        if partial_result:
                            self.publish_result(partial_result, False)
        except Exception as e:
            rospy.logfatal("Error: %s", str(e))
        except KeyboardInterrupt:
            rospy.loginfo("Stopping the VOSK speech recognition node...")
            rospy.sleep(1)
    
    def publish_result(self, text, is_final):
        self.msg.isSpeech_recognized = is_final
        self.msg.time_recognized = rospy.Time.now()
        self.msg.final_result = text if is_final else "unk"
        self.msg.partial_result = "unk" if is_final else text
        self.pub_vosk.publish(self.msg)
        rospy.sleep(0.1)
        (self.pub_final if is_final else self.pub_partial).publish(text)

if __name__ == '__main__':
    try:
        recognizer = VoskSR()
        recognizer.speech_recognize()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        rospy.logfatal("Error occurred! Stopping the vosk speech recognition node...")
        rospy.sleep(1)

#!/usr/bin/env python
import os
import sounddevice as sd
import soundfile as sf
import nemo.collections.asr as nemo_asr

import rospy
from std_msgs.msg import String

RATE = 16000
CHANNELS = 1
# Change model name & languages here.
MODEL_NAME = {
    'ko':"cwwojin/stt_kr_conformer_ctc_small_20",
    'en':"stt_en_conformer_ctc_small",
}

class NemoAgent(object):
    """Speech Recognize using Nvidia NeMo"""
    def __init__(self, lang, frame, device="cpu", topic="speech_recognition"):
        #set sounddevice defaults
        sd.default.samplerate = RATE
        sd.default.channels = CHANNELS
        
        #start ROS node
        rospy.init_node('nemo_node')
        rospy.on_shutdown(self.shutdown)
        self.pub = rospy.Publisher(topic, String, queue_size=10)
        
        #config
        self.lang = lang
        self.frame = frame
        try :
            self.model_name = MODEL_NAME[lang]
        except KeyError :
            rospy.loginfo(f"unknown language {lang} (supported languages : en, ko)")
        
        #load model from HuggingFace
        self.model = nemo_asr.models.ASRModel.from_pretrained(model_name=self.model_name, map_location=device)
        self.model.eval();

        #audio path
        self.audio_path = "out.wav"

        #inputs
        self.inputs = ['y','n','c']

    def record(self):
        rospy.loginfo("Recording audio..")
        mydata = sd.rec(int(RATE * self.frame),blocking=True)
        sf.write(self.audio_path, mydata, RATE)
        # rospy.loginfo("recording done.")

    def transcribe(self):
        #transcribe ONE file
        return self.model.transcribe([self.audio_path])[0]

    def shutdown(self):
        #self.stop()
        if os.path.isfile(self.audio_path) :
            os.remove(self.audio_path)
        rospy.loginfo("nemo-recognizer shutdown..")
        
    def recognize_speech(self):
        rate = rospy.Rate(6)

        while not rospy.is_shutdown() :
            #get keyboard input
            command = str(input(f"[INPUT] 'y' : record for {self.frame} seconds / 'c' : cli input / 'n' : shutdown  "))

            if command == 'y' :
                self.record()
                result = self.transcribe()
                self.pub.publish(String(result))
                rospy.loginfo(f"Transcription : {result}")
            elif command == 'c' :
                result = input("command: ")
                self.pub.publish(String(result))
                rospy.loginfo(f"CLI-input : {result}")
            elif command == 'n' :
                break
            rate.sleep()

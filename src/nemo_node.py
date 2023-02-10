#!/usr/bin/env python
import rospy
import rosparam
import torch

from utils.agent import NemoAgent

def main() :
    lang = rosparam.get_param('lang')
    frame = rosparam.get_param('frame')
    topic = rosparam.get_param('speech_channel')
    agent = NemoAgent(lang, frame, device="cuda" if torch.cuda.is_available() else "cpu", topic=topic)
    agent.recognize_speech()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException :
        pass
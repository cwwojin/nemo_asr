#!/usr/bin/env python
import rospy
import rosparam
import torch

from utils.agent import NemoAgent

def main() :
    lang = rosparam.get_param('lang')
    frame = rosparam.get_param('frame')
    agent = NemoAgent(lang, frame, device="cuda" if torch.cuda.is_available() else "cpu")
    agent.recognize_speech()
    #rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException :
        pass
    # except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
    #     rospy.logfatal("Stopping nemo-node...")
    #     rospy.sleep(1)
    #     print("node terminated")
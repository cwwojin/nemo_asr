#!/usr/bin/env python
import rospy
import rosparam
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " heard : %s", data.data)
    
def listener():
    rospy.init_node('listener', anonymous=True)
    #topic = rosparam.get_param('speech_channel')
    rospy.Subscriber("speech_recognition", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
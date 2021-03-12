#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    print data.split(',')

def VicParkListener():
    rospy.init_node("VicParkListener", anonymous=True)
    rospy.Subscriber("laser", string, callback)
    rospy.spin()
if __name__ == "__main__":
    VicParkListener()

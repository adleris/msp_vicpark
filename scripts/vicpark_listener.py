#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import ast

def callback(data):
    # ugly but it works: parse the string representation of the list of tuples
    # back into the appropriate data structure
    #
    # the strings being passed by ROS  have line breaks that have to be removed, 
    # and there is a 'data: ' at the start too
    data = str(data).replace('data: ','').replace('\\\n  \\','')#.replace('\n','').replace('\\','')
    # we also need to scrub quotation marks at the start and end of the string 
    # which are causing it to get parsed asa string not a list
    # (this took a long time to figure out)
    datalist =  ast.literal_eval(data[1:-1])

    print datalist

def VicParkListener():
    rospy.init_node("VicParkListener", anonymous=True)
    rospy.Subscriber("landmarks", String, callback)
    rospy.spin()
if __name__ == "__main__":
    VicParkListener()

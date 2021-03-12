#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import ast

def lm_callback(data):
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

    print "<---- LANDMARK READ ---->" + " (success)"
    #print datalist

# def gps_callback(data):
#     """to parse gps data as provided by a String message type"""
#     data = str(data)[5:-1].replace('"','').strip().replace("[ ","[").replace(" ",",")
#     data = ast.literal_eval(data)
#     print  "<==== GPS READ ====>"
#     print  data

def gps_callback(data):
    print  "<==== GPS READ ====>" + " (success: [{0}, {1}])".format(data.pose.pose.position.x, data.pose.pose.position.y)

def lsr_callback(data):
    print "<:::: LASER READ ::::>" + " (success: {} values)".format(len(data.ranges))

def VicParkListener():
    rospy.init_node("VicParkListener", anonymous=True)
    rospy.Subscriber("landmarks", String, lm_callback)
    rospy.Subscriber("gps", Odometry, gps_callback)
    rospy.Subscriber("lsr", LaserScan, lsr_callback)
    rospy.spin()
if __name__ == "__main__":
    VicParkListener()

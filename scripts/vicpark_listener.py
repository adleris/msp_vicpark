#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import matplotlib.pyplot as plt
import ast

def ekf_cb(data):
    EKFP.xbuffer.append(data.pose.pose.position.x)
    EKFP.ybuffer.append(data.pose.pose.position.y)
    EKFP._plot_buffer()

class EKF_Printer():
    def __init__(self):
        self.xbuffer = []
        self.ybuffer = []
        plt.ion()
    def _plot_buffer(self):
        plt.plot(self.xbuffer, self.ybuffer, '.')
        plt.draw()
        plt.pause(0.001)
    

def VicParkListener():
    
    rospy.init_node("VicParkListener", anonymous=True)
    rospy.Subscriber('odom_ekf', Odometry, ekf_cb)
    rospy.spin()

if __name__ == "__main__":
    EKFP = EKF_Printer()
    VicParkListener()

#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseWithCovariance,Point
from nav_msgs.msg import Odometry
from common.msg import GPSData
import numpy as np


class Republisher():
    START_LAT = -37.910515
    START_LON =  145.134948333
    ONE_DEGREE_LAT_METRES =  111132.954

    def __init__(self):
        rospy.init_node('novarover_republisher', anonymous=True)

        self.pub = rospy.Publisher('/republisher/gps_data', Odometry, queue_size=1)

    def callback(self, data):
        # package up our message into a type readable by RViz
        odom = Odometry()

        odom.header = Header()
        odom.header.seq = 0
        odom.header.frame_id = "base_link"

        odom.child_frame_id = "odom"

        odom.pose = PoseWithCovariance()
        odom.pose.pose = Pose()
        odom.pose.pose.position = Point()

        # get lat and long distance from "start" and convert to displacement in m
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.lat_lon_to_m_m(
                                                                data.latitude, data.longitude)

        self.pub.publish(odom)
    
    def run(self):
        rospy.Subscriber('/rover/gps_data', GPSData, self.callback)
        rospy.spin()

    def lat_lon_to_m_m(self, lat, lon):
        """https://stackoverflow.com/a/19356480"""
        lat_dist = (lat - self.START_LAT)
        lon_dist = (lon - self.START_LON)
        # return (lat_dist * self.ONE_DEGREE_LAT_METRES, lon_dist * self.ONE_DEGREE_LAT_METRES)

        # more accurate calculation to take into account that we aren't on the equator
        lat_mid = (lat + self.START_LAT)/2.0
        m_per_degree_lat = 111132.954 - 559.822 * np.cos(2.0 * lat_mid) + 1.175*np.cos(4.0 * lat_mid)
        m_per_degree_lon = (3.141592653359/180) * 6367449 * np.cos(lat_mid)

        return lat_dist * m_per_degree_lat, lon_dist*m_per_degree_lon

        




if __name__ == '__main__':
    r = Republisher()
    r.run()
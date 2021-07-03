#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_msgs.msg
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseWithCovariance,Point, Transform, TransformStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from common.msg import GPSData
import numpy as np


class Republisher():
    START_LAT = -37.910515
    START_LON =  145.134948333
    ONE_DEGREE_LAT_METRES =  111132.954

    def __init__(self):
        rospy.init_node('novarover_republisher', anonymous=True)

        self.pub_gps = rospy.Publisher('/republisher/gps_data', Odometry, queue_size=1)
        self.pub_imu = rospy.Publisher('/republisher/imu_data', Imu, queue_size=1)
#	self.pub_mag = rospy.Publisher('republisher/mag_data', tf2_msgs.msg.TFMessage, queue_size=1)
	self.tf_br = tf2_ros.TransformBroadcaster()

    def gps_callback(self, data):
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
        odom.pose.pose.position.y, odom.pose.pose.position.x = self.lat_lon_to_m_m(
                                                                data.latitude, data.longitude)

        self.pub_gps.publish(odom)
    
    def imu_callback(self, data):
        """
        republish the imu data to a new topic with a different TF frame
        """
        data.header.frame_id = "imu_link"
        self.pub_imu.publish(data)

    def mag_callback(self, imu_data):
	"""
	subscribe to an Imu message and republish the orientation quaternion within
	frame is set to base link
	"""
	result = TransformStamped()
	result.transform = Transform()
	result.transform.rotation = imu_data.orientation
	result.transform.translation = Vector3()
	result.transform.translation.x = np.random.rand()	# let's see if things move around
	result.header.frame_id = "magnetometer_link"
	result.header.stamp = rospy.Time.now()

#	tfm = tf2_msgs.msg.TFMessage([result])
#	self.pub_mag.publish(tfm)
	self.tf_br.sendTransform(result)

    def run(self):
        rospy.Subscriber('/rover/gps_data', GPSData, self.gps_callback)
        rospy.Subscriber('/rover/raw_imu0', Imu, self.imu_callback)
        rospy.Subscriber('/rover/raw_imu0', Imu, self.mag_callback)
        rospy.spin()

    def lat_lon_to_m_m(self, lat, lon):
        """https://stackoverflow.com/a/19356480"""
        lat_dist = (lat - self.START_LAT)
        lon_dist = (lon - self.START_LON)
        # return (lat_dist * self.ONE_DEGREE_LAT_METRES, lon_dist * self.ONE_DEGREE_LAT_METRES)

        # more accurate calculation to take into account that we aren't on the equator
        lat_mid = (lat + self.START_LAT)/2.0
        m_per_degree_lat = 111132.954 - 559.822 * np.cos(2.0 * lat_mid) + 1.175*np.cos(4.0 * lat_mid)
        m_per_degree_lon = (np.pi/180) * 6367449 * np.cos(lat_mid)

        return lat_dist * m_per_degree_lat, lon_dist*m_per_degree_lon

        


if __name__ == '__main__':
    r = Republisher()
    r.run()

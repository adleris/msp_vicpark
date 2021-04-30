#!/usr/bin/env python
import numpy as np

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseWithCovariance, Twist, TwistWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import MatlabImporter
from motion_model import ackerman_model

PI_ON_2 = 1.57079632679
# RATE = 1
RATE = 0.214

def VicParkPublisher():
    MI = MatlabImporter.MatlabImporter()
    pub_lm  = rospy.Publisher('VicPark/landmarks', String, queue_size=1)
    pub_gps = rospy.Publisher('VicPark/gps', Odometry, queue_size=1)
    pub_dr  = rospy.Publisher('VicPark/dr', Odometry, queue_size=1)
    pub_lsr = rospy.Publisher('VicPark/scan', LaserScan, queue_size=1)
    pub_odom= rospy.Publisher('VicPark/odom', Odometry, queue_size=1)
    
    rospy.init_node('VicPark', anonymous=True)
    rate = rospy.Rate(1/RATE)

    # publish to all topics
    while not rospy.is_shutdown():

        # generate landmark odometry
        lm_data = MI.next_lm()
        rospy.loginfo(lm_data)
        pub_lm.publish(str(lm_data))


        # generate gps odometry
        gps_pose = Pose()
        gps_pose_with_cov = PoseWithCovariance()
        gps_pose_with_cov.pose = gps_pose
        gps_odom = Odometry()
        gps_odom.pose = gps_pose_with_cov

        gps_data = MI.next_gps()
        gps_pose.position.x = gps_data[1]
        gps_pose.position.y = gps_data[2]
        # rospy.loginfo(gps_odom)
        pub_gps.publish(gps_odom)


        # # generate DR odometry
        # dr_data = MI.next_dr()
        # # rospy.loginfo(dr_data)
        # pub_dr.publish(str(dr_data))

        # generate dr odometry
        dr_twist = Twist()
        dr_twist_with_cov = TwistWithCovariance()
        dr_twist_with_cov.twist = dr_twist
        dr_odom = Odometry()
        dr_odom.twist = dr_twist_with_cov

        dr_data = MI.next_dr()
        dx,dy,dtheta = ackerman_model(dr_data[2], dr_data[1], RATE)
        dr_twist.linear.x  = dx/RATE
        dr_twist.linear.y  = dy/RATE
        dr_twist.angular.x = dtheta/RATE
        dr_twist.angular.y = dtheta/RATE
        # rospy.loginfo(dr_odom)
        pub_dr.publish(dr_odom)


        # generate Laser scans
        # set up LaserScan object
        ls = LaserScan()
        ls.angle_min = -PI_ON_2
        ls.angle_max = PI_ON_2
        ls.angle_increment = 0.5
        ls.range_max = 80.0
        ls.range_min = 0.0
        ls.time_increment = 0.214       # based on differences in TLsr data file
        ls.scan_time = 0.214            # based on differences in TLsr data file

        ls.ranges = MI.next_lsr()
        pub_lsr.publish(ls)


        # splice together gps and dr into a single odometry message
        full_odom = Odometry()
        full_odom.twist = dr_twist_with_cov
        full_odom.pose = gps_pose_with_cov
        pub_odom.publish(full_odom)


        rate.sleep()


if __name__ == '__main__':
    try:
        VicParkPublisher()
    except rospy.ROSInterruptException:
        pass

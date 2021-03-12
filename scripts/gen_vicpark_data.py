#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import MatlabImporter

PI_ON_2 = 1.57079632679

def VicParkPublisher():
    MI = MatlabImporter.MatlabImporter()
    pub_lm = rospy.Publisher('landmarks', String, queue_size=1)
    pub_gps   = rospy.Publisher('gps', Odometry, queue_size=1)
    pub_dr    = rospy.Publisher('dr', String, queue_size=1)
    pub_lsr   = rospy.Publisher('lsr', LaserScan, queue_size=1)
    
    rospy.init_node('VicPark', anonymous=True)
    rate = rospy.Rate(1)

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


        # generate DR odometry
        dr_data = MI.next_dr()
        # rospy.loginfo(dr_data)
        pub_dr.publish(str(dr_data))


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


        rate.sleep()


if __name__ == '__main__':
    try:
        VicParkPublisher()
    except rospy.ROSInterruptException:
        pass

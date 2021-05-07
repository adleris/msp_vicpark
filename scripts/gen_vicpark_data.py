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

class CallbackController():
    """
    Act as publisher for the vicpark data, at varying rates
    """
    TEST_RATE = 0.2
    TIMESTEP = TEST_RATE * 100  # I think this works based on the values in the time files, but I'm not 100%
                                # do we need to sample faster?

    def __init__(self):
        self.MI = MatlabImporter.MatlabImporter()

        # store the next value to be generated so we can come back to it
        # self.next_lm   = self.MI.next_lm()
        self.next_gps  = self.MI.next_gps()
        self.next_dr   = self.MI.next_dr()
        self.next_lsr  = self.MI.next_lsr()

        # initialise to starting times in the dataset
        # self.lm_timer   = self.next_lm[0]  
        self.gps_timer  = self.next_gps[0]
        self.dr_timer   = self.next_dr[0]
        self.lsr_timer  = self.next_lsr[0]

        # for full odometry
        self.dr_twist_with_cov = None
        self.gps_pose_with_cov = None

    def run(self):
        """
        loop controller, iterates each callback function with period TEST_RATE to see if they're ready to publish
        """
        # self.pub_lm  = rospy.Publisher('VicPark/landmarks', String, queue_size=1)
        self.pub_gps = rospy.Publisher('VicPark/gps', Odometry, queue_size=1)
        self.pub_dr  = rospy.Publisher('VicPark/dr', Odometry, queue_size=1)
        self.pub_lsr = rospy.Publisher('VicPark/scan', LaserScan, queue_size=1)
        self.pub_odom= rospy.Publisher('VicPark/odom', Odometry, queue_size=1)

        rospy.init_node('VicPark', anonymous=True)

        rospy.Timer(rospy.Duration(self.TEST_RATE), self.dr)
        rospy.Timer(rospy.Duration(self.TEST_RATE), self.gps)
        # rospy.Timer(rospy.Duration(1.0 / self.TEST_RATE), self.lm)
        rospy.Timer(rospy.Duration(self.TEST_RATE), self.lsr)

        rospy.spin()


    # define the callbacks for each data stream
    def dr(self, event):
        self.dr_timer += self.TIMESTEP
        if self.next_dr[0] <= self.dr_timer:
            next_publish = self.next_dr
            self.next_dr = self.MI.next_dr()

            # generate dr odometry
            dr_twist = Twist()
            dr_twist_with_cov = TwistWithCovariance()
            dr_twist_with_cov.twist = dr_twist
            dr_odom = Odometry()
            dr_odom.twist = dr_twist_with_cov

            self.dr_twist_with_cov = dr_twist_with_cov      # for full odom

            dt = self.next_dr[0] - next_publish[0]
            dx,dy,dtheta = ackerman_model(next_publish[2], next_publish[1], dt)
            dr_twist.linear.x  = dx/dt
            dr_twist.linear.y  = dy/dt
            dr_twist.angular.x = dtheta/dt
            dr_twist.angular.y = dtheta/dt

            self.pub_dr.publish(dr_odom)

            self.odom()


    def gps(self, event):
        self.gps_timer += self.TIMESTEP
        if float(self.next_gps[0]) < self.gps_timer:
            next_publish = self.next_gps
            self.next_gps = self.MI.next_gps()

            # generate gps odometry
            gps_pose = Pose()
            gps_pose_with_cov = PoseWithCovariance()
            gps_pose_with_cov.pose = gps_pose
            gps_odom = Odometry()
            gps_odom.pose = gps_pose_with_cov

            self.gps_pose_with_cov = gps_pose_with_cov      # for full odom

            gps_pose.position.x = next_publish[1]
            gps_pose.position.y = next_publish[2]

            self.pub_gps.publish(gps_odom)

            self.odom()


    def lm(self, event):
        """
        broken as matlab data doesn't exportwith timestamp
        """
        self.lm_timer += self.TIMESTEP
        if self.next_lm[0] <= self.lm_timer:
            next_publish = self.next_lm
            self.next_lm = self.MI.next_lm()

            # publish landmarks 
            self.pub_lm.publish(str(next_publish))

    
    def lsr(self, event):
        self.lsr_timer += self.TIMESTEP
        if self.next_lsr[0] <= self.lsr_timer:
            next_publish = self.next_lsr
            self.next_lsr = self.MI.next_lsr()

            dt = self.next_lsr[0] - next_publish[0]

            # generate Laser scans
            # set up LaserScan object
            ls = LaserScan()
            ls.angle_min = -PI_ON_2
            ls.angle_max = PI_ON_2
            ls.angle_increment = 0.5
            ls.range_max = 80.0
            ls.range_min = 0.0
            ls.time_increment = dt
            ls.scan_time = dt

            ls.ranges = next_publish
            self.pub_lsr.publish(ls)


    def odom(self):
        """
        grab stored data from gps and dr and put into a signle published object
        kinda works
        """
        if self.dr_twist_with_cov is not None and self.gps_pose_with_cov is not None:
            # splice together gps and dr into a single odometry message
            full_odom = Odometry()
            full_odom.twist = self.dr_twist_with_cov
            full_odom.pose = self.gps_pose_with_cov
            self.pub_odom.publish(full_odom)




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
        c = CallbackController()
        c.run()
        # VicParkPublisher()
    except rospy.ROSInterruptException:
        pass

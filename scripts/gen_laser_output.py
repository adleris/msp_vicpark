#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import MatlabImporter

def VicParkPublisher():
    MI = MatlabImporter()


    pub = rospy.Publisher('vic_park_data', String, queue_size=10)
    rospy.init_node('VicPark', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        data = VicParkImporter.get_next_laser_string()
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()


if __name__ == '__main__':
    try:
        VicParkPublisher()
    except rospy.ROSInterruptException:
        pass

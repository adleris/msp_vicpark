#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import MatlabImporter

def VicParkPublisher():
    MI = MatlabImporter.MatlabImporter()
    pub_lm = rospy.Publisher('landmarks', String, queue_size=10)
    pub_gps   = rospy.Publisher('gps', String, queue_size=10)
    pub_dr    = rospy.Publisher('dr', String, queue_size=10)
    
    rospy.init_node('VicPark', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # publish to all topics
        data = MI.next_lm()
        rospy.loginfo(data)
        pub_lm.publish(str(data))

        data = MI.next_gps()
        rospy.loginfo(data)
        pub_gps.publish(str(data))

        data = MI.next_dr()
        rospy.loginfo(data)
        pub_dr.publish(str(data))

        rate.sleep()


if __name__ == '__main__':
    try:
        VicParkPublisher()
    except rospy.ROSInterruptException:
        pass

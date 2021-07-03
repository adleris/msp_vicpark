#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg
import numpy as np

if __name__ == "__main__":
	rospy.init_node('magnetometer_broadcaster')
	br = tf2_ros.TransformBroadcaster()
	t = geometry_msgs.msg.TransformStamped()
	t.header.frame_id = "base_link"
	t.child_frame_id = "magnetometer_link"

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		t.header.stamp = rospy.Time.now()
		t.transform.translation.x = np.sin(rospy.Time.now().to_sec() * np.pi)

		br.sendTransform(t)
		rate.sleep()

# RTABMAP_ROS
See [here](wiki.ros.org/rtabmap_ros) for more documentation (section 4 onwards)

In particular, section 4.3 contains information about using LaserScans (and icp_odometry in section 4.6, if that's needed)

## rtabmap settings

comment/uncomment the MAPPING/LOCALISATION lines in the launch/rtabmap_launch.launch file to change mode
aguments:
	"--delete_db_on_start" or "-d" to start a new session, otherwise continues from previous session

## Using 2D Laser + Wheel Odometry
See [section 2.4 and 2.5 here](wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot#A2D_laser_only) for instructions of how to set up laser and wheel odometry with no camera dependence. This is out next step!

## Using RTABMAP with camera
wiki.ros.org/rtabmap_ros/Tutorials/HandHeldMapping
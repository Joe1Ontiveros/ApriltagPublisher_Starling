# ApriltagPublisher_Starling
Uses C++ to parse the voxl-inspect-tags output, in order to convert it into a ROS topic (PoseStamped)

# Setup:
Copy-paste the mavscripts package/folder into your ../mavros_test/catkin_ws/src/ folder and run catkin build in ../mavros_test/catkin_ws/ 

source /devel/setup.bash after building and run roslaunch mavscripts apriltag_ros.launch


Publishes as "mavros/local_position/april_offset" to match MAVROS outputs

Only tested to work on the VOXL Starling, ill work on a better fix/merge into the MODALAI eventually! 

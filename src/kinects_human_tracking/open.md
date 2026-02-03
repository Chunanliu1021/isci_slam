# how to open
- rosbag play -l velodyne_point-no_tf.bag
- roslaunch kinects_human_tracking kinect_merge.launch
- roslaunch kinects_human_tracking closest_pt_tracking.launch
- rviz
- rostopic echo /kinect_merge/closest_vel_tracking
- rostopic echo /kinect_merge/closest_pt_tracking
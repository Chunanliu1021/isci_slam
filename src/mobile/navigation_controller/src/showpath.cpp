#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

main (int argc, char **argv)
{
	ros::init (argc, argv, "show_path");

	ros::NodeHandle ph;
	ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("trajectory",1, true);
	tf::TransformListener listener;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	nav_msgs::Path path;

	path.header.stamp=current_time;
	path.header.frame_id="map";

	ros::Rate loop_rate(10);
	while (ros::ok()) {

		current_time = ros::Time::now();

		//get laser position
		tf::StampedTransform transform;
		tf::Quaternion q;
		try{
			listener.waitForTransform("/base_footprint", "/world", ros::Time(0), ros::Duration(10.0) );
			listener.lookupTransform("/world", "/base_footprint", ros::Time(0), transform);
			q = transform.getRotation(); 
		}
		catch (tf::TransformException ex){	
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}

		geometry_msgs::PoseStamped this_pose_stamped;
		this_pose_stamped.pose.position.x = transform.getOrigin().x();
		this_pose_stamped.pose.position.y = transform.getOrigin().y();

		geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(tf::getYaw(q));
		this_pose_stamped.pose.orientation.x = goal_quat.x;
		this_pose_stamped.pose.orientation.y = goal_quat.y;
		this_pose_stamped.pose.orientation.z = goal_quat.z;
		this_pose_stamped.pose.orientation.w = goal_quat.w;

		this_pose_stamped.header.stamp=current_time;
		this_pose_stamped.header.frame_id="robot_path";
		path.poses.push_back(this_pose_stamped);

		path_pub.publish(path);
		
		last_time = current_time;
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

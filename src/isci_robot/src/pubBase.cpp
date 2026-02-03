#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#include <thread>
#include <fstream>
using namespace std;
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
ros::Publisher marker_pub;
visualization_msgs::Marker line_strip;
double cmd_x = 0;
double cmd_y = 0;
double cmd_z = 0;
double cmd_pitch = 0;
double cmd_roll = 0;
double cmd_yaw = 0;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  
  
  ros::NodeHandle n;
  
  tf::TransformListener listener;
  listener.waitForTransform("/odom", "/base_link",ros::Time::now(), ros::Duration(1.0));
  ros::Publisher basePub = n.advertise<geometry_msgs::Pose>("/base_pose", 1, false);
  ros::Time::now();
  // std::thread t1(TPCallback);

  tf::StampedTransform transform;
  // tf::TransformListener listener;
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
     try{
        listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);

        std::cout << "x,y,z is : " << transform.getOrigin().x() <<  ", " << transform.getOrigin().y() << ", " << transform.getOrigin().z()<<std::endl;

        geometry_msgs::Pose pubmsg;
        pubmsg.position.x = transform.getOrigin().x();
        pubmsg.position.y = transform.getOrigin().y();
        pubmsg.position.z = 0;
        pubmsg.orientation.x = transform.getRotation().x();
        pubmsg.orientation.y = transform.getRotation().y();
        pubmsg.orientation.z = transform.getRotation().z();
        pubmsg.orientation.w = transform.getRotation().w();
        basePub.publish(pubmsg);
      
      }catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
      }
    ros::spinOnce();
    loop_rate.sleep();
  }  


// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%
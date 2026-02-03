#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <fstream>

using namespace std;

ros::Publisher marker_pub;
visualization_msgs::Marker line_strip;

void add_path(float x, float y, float z){
  line_strip.header.frame_id =  "map";
  line_strip.header.stamp =  ros::Time::now();
  line_strip.ns = "points_and_lines";
  line_strip.action  = visualization_msgs::Marker::ADD;
  line_strip.lifetime = ros::Duration();
  line_strip.pose.orientation.w =1.0;
  line_strip.id = 1;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.01;

  // Line strip is green
  line_strip.color.g = 1.0;
  line_strip.color.a = 1.0;

  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  
  line_strip.points.push_back(p);
  marker_pub.publish(line_strip);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "record");
    ros::NodeHandle nh;
    ros::Time ros_first_time;

    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    marker_pub = nh.advertise<visualization_msgs::Marker>("car_trajectory", 10);

    fstream base_link_trajectory;
    base_link_trajectory.open("/home/caliiu/ws_hdl_slam/src/hdl_localization/output/car_trajectory.xls",std::ofstream::out | std::ofstream::trunc);

    double x, y, z;
    double roll, pitch, yaw;

    ros_first_time = ros::Time::now();
    ros::Rate loop_rate(20);
    cout << "recording..." << endl;
    while (ros::ok())
    {
        try{
            listener.waitForTransform("/map","/base_link", ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
            tf::Quaternion q(transform.getRotation().x(),
                            transform.getRotation().y(),
                            transform.getRotation().z(),
                            transform.getRotation().w());
            tf::Matrix3x3 m(q);
            m.getRPY(roll,pitch,yaw);

            x = transform.getOrigin().x();
            y = transform.getOrigin().y();
            z = transform.getOrigin().z();

            auto time = ros::Time::now().toSec()-ros_first_time.toSec();
            base_link_trajectory << time << "\t" << x << "\t" << y << "\t" << z << "\t" << roll << "\t" << pitch << "\t" <<yaw <<"\n";
            
            add_path(x, y, z);
                        
        }catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
        loop_rate.sleep();
    }

    return 0;
}
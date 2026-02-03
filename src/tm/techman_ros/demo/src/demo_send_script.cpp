// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>

// std header
#include <sstream>
#include <cstdlib>

// TM Driver header
#include "tm_msgs/SendScript.h"

int main(int argc, char **argv)
{    
  std::string cmd1 = "ContinueVJog()";
  std::string cmd2 = "SetContinueVJog(0.5, 0.5, 0.5, 0.5, 0.5, 0.5)";
  std::string cmd3 = "StopContinueVmode()";

  ros::init(argc, argv, "demo_send_script");      
  ros::NodeHandle nh_demo; 
  ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
  tm_msgs::SendScript srv;
    
  //Request	
  srv.request.id = "demo";
  srv.request.script = cmd1;

  ROS_INFO("Joint velocity mode on");

  if (client.call(srv))                             
  {
    if (srv.response.ok) ROS_INFO_STREAM("Sent script to robot");
    else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
  }
  else
  {
    ROS_ERROR_STREAM("Error send script to robot");
    return 1;
  }

  srv.request.id = "go";
  srv.request.script = cmd2;

  if (client.call(srv))                             
  {
    if (srv.response.ok) ROS_INFO_STREAM("Sent script to robot");
    else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
  }
  else
  {
    ROS_ERROR_STREAM("Error send script to robot");
    return 1;
  }

  ROS_INFO("Wait for 5 minutes");

  sleep(5);

  srv.request.id = "stop";
  srv.request.script = cmd3;

  ROS_INFO("Joint velocity mode off");

  if (client.call(srv))                             
  {
    if (srv.response.ok) ROS_INFO_STREAM("Sent script to robot");
    else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
  }
  else
  {
    ROS_ERROR_STREAM("Error send script to robot");
    return 1;
  }

  //ROS_INFO_STREAM_NAMED("demo_sendscript", "shutdown.");
  return 0;

}

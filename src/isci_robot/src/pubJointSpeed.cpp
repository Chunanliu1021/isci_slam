#include"ros/ros.h" 
#include"std_msgs/Float64MultiArray.h"          
#include"std_msgs/String.h" 
#include<sstream>
#include<vector>
#include<string.h> 
using namespace std; 



int main(int argc, char** argv) 
{ 	
	ros::init(argc, argv, "pubJointSpeed"); // 节点名称 
	ros::NodeHandle n; 
	ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("arm_controller/command", 1000); 
	//ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter",1000); 

  while (ros::ok() && chatter_pub.getNumSubscribers() == 0) {
    ros::Rate(100).sleep();
  }

	ros::Rate loop_rate(10); 
	int count = 0; 
	std_msgs::Float64MultiArray msg;
  vector<double> array_test={0.1,0.05,0.1,0.1,0.1,0.1};

  msg.data = array_test;
  chatter_pub.publish(msg); 
  printf("Done\n");
	return 0; 
}


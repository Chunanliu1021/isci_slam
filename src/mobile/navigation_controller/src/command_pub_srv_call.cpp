#include "ros/ros.h"
#include <std_msgs/String.h>
#include "navigation_controller/AgvNav.h"
#include <iostream>
#include <fstream>

class CommandPubSrvCall {
public:
	CommandPubSrvCall();
	~CommandPubSrvCall();

	void get_call();
private:
	ros::NodeHandle nh_;
	ros::ServiceClient client_;

};

CommandPubSrvCall::CommandPubSrvCall()
{
	client_ = nh_.serviceClient<navigation_controller::AgvNav>("pos_call");
}

CommandPubSrvCall::~CommandPubSrvCall()
{
	
}


void CommandPubSrvCall::get_call()
{

	std::vector<float> start(3,0.0);
	std::vector<float> end(3,0.0);

	navigation_controller::AgvNav srv;

	while (ros::ok()) {
		

		ROS_INFO("start : ");
		scanf("%f %f %f",&start[0],&start[1],&start[2]);
		ROS_INFO("end : ");
		scanf("%f %f %f",&end[0],&end[1],&end[2]);


		srv.request.start.assign(start.begin(),start.end());
		srv.request.end.assign(end.begin(),end.end());
		

		if(client_.call(srv)) {
			ROS_INFO("call service success");
		}
		else {
			ROS_INFO("call service fail");
		}
		
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "command_pub_srv_call");
	CommandPubSrvCall commandpubsrvcall;

	commandpubsrvcall.get_call();

	ros::spin();	
	return 0;
}

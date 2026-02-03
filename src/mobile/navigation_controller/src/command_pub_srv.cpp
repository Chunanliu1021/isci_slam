#include "ros/ros.h"
#include <std_msgs/String.h>
#include "navigation_controller/AgvNav.h"
#include "navigation_controller/command.h"
#include <iostream>
#include <fstream>

class CommandPubSrv {
public:
	CommandPubSrv();
	~CommandPubSrv();
	bool get_place(navigation_controller::AgvNav::Request &req,navigation_controller::AgvNav::Response &res);

private:

	ros::NodeHandle nh_;
	ros::ServiceClient client_;

};

CommandPubSrv::CommandPubSrv()
{
	client_ = nh_.serviceClient<navigation_controller::command>("pos_cmd");

}

CommandPubSrv::~CommandPubSrv()
{
	
}


bool CommandPubSrv::get_place(navigation_controller::AgvNav::Request &req,navigation_controller::AgvNav::Response &res)
{

	int step = 2;
	int num = 0;
	navigation_controller::command srv;

	while (step > 0){
		num = 2;		

		while(num > 0){
			
			if(step==2){
				srv.request.type = num;
				srv.request.x = req.start[0];
				srv.request.y = req.start[1];
				srv.request.theta = req.start[2];
			}			
			else{
				srv.request.type = num;
				srv.request.x = req.end[0];
				srv.request.y = req.end[1];
				srv.request.theta = req.end[2];
			}

			if(client_.call(srv)) {
				ROS_INFO("call service success");
				srv.request.type = 0;

				while(srv.response.run_completed == false) {				
					usleep(100000);
					//ROS_INFO("hello servie");
					client_.call(srv);
				}
				num --;
			}
			else {
				ROS_INFO("call service fail");
				res.status = false;
				res.error_msg = "call service fail";
			}
		}
		step--;
	}

	res.status = true;
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "command_pub_srv");
	ros::NodeHandle nh;
	CommandPubSrv commandpubsrv;


	ros::ServiceServer service = nh.advertiseService("pos_call", &CommandPubSrv::get_place, &commandpubsrv);

	ros::Rate rate(10);
	ros::spin();	
	return 0;
}

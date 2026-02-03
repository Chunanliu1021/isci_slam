// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>

// std header
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <sys/time.h>
#include <iostream>
#include <termios.h>
#include <math.h>
#include <sstream>
#include <cstdlib>

// TM Driver header
#include "tm_driver/tm_print.h"
#include "tm_msgs/ConnectTM.h"
#include "tm_msgs/SendScript.h"
#include "tm_msgs/SetEvent.h"
#include "tm_msgs/SetPositions.h"
#include "tm_msgs/SetVelocity.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "demo_ui");      
	ros::NodeHandle nh_demo; 

	ros::ServiceClient vel_client = nh_demo.serviceClient<tm_msgs::SetVelocity>("tm_driver/set_velocity");
	ros::ServiceClient pos_client = nh_demo.serviceClient<tm_msgs::SetPositions>("tm_driver/set_positions");
	ros::ServiceClient event_client = nh_demo.serviceClient<tm_msgs::SetEvent>("tm_driver/set_event");
	ros::ServiceClient script_client = nh_demo.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
	ros::ServiceClient connect_client = nh_demo.serviceClient<tm_msgs::ConnectTM>("tm_diver/connect_tm");

	tm_msgs::SetPositions pos_srv;
	tm_msgs::SetVelocity vel_srv;
	tm_msgs::SetEvent event_srv;
	tm_msgs::SendScript script_srv;
	tm_msgs::ConnectTM connect_srv;

	int id_cnt = 0;

	char cstr[512];
	char delim[] = " ,;\t";
	char c;
	while (true) {
		memset(cstr, 0, 512);
		fgets(cstr, 512, stdin);
		int n = int(strlen(cstr));
		if (n > 0) {
			if (cstr[n - 1] == '\n') { cstr[n - 1] = '\0'; }
		}

		if (strncmp(cstr, "quit", 4) == 0) {
			std::string cmd = "ScriptExit()";
			script_srv.request.id = "quit";
  			script_srv.request.script = cmd;
  			if (script_client.call(script_srv))                             
			{
				if (script_srv.response.ok) ROS_INFO_STREAM("Exit script to robot");
				else ROS_WARN_STREAM("Exit script to robot , but response not yet ok ");
			}
			else
			{
				ROS_ERROR_STREAM("Error Exit script to robot");
				return 1;
			}    	

			ROS_INFO_STREAM_NAMED("ScriptExit", "shutdown.");
			break;
		}
		else if (strncmp(cstr, "start", 5) == 0) {
			connect_srv.request.server = connect_srv.request.TMSVR;
			connect_srv.request.reconnect = true;
			connect_srv.request.timeout = 0;
			connect_srv.request.timeval = 0;
			if (connect_client.call(connect_srv))                             
			{
				if (connect_srv.response.ok) ROS_INFO_STREAM("ConnectTM to robot");
				else ROS_WARN_STREAM("ConnectTM to robot , but response not yet ok ");
			}
			else
			{
				ROS_ERROR_STREAM("Error ConnectTM to robot");
				return 1;
			}
		}
		else if (strncmp(cstr, "stop", 4) == 0) {
			std::string cmd = "StopAndClearBuffer()";
			script_srv.request.id = "stop";
  			script_srv.request.script = cmd;
  			if (script_client.call(script_srv))                             
			{
				if (script_srv.response.ok) ROS_INFO_STREAM("Exit script to robot");
				else ROS_WARN_STREAM("Exit script to robot , but response not yet ok ");
			}
			else
			{
				ROS_ERROR_STREAM("Error Exit script to robot");
				return 1;
			}    	

			ROS_INFO_STREAM_NAMED("ScriptExit", "shutdown.");
			break;
		}
		else if (strncmp(cstr, "home", 4) == 0) {
			double blend = 0;
            std::vector<double> vec1 = {0.0,    0.0,    0.0,    0.0,    0.0,    0.0};
            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            ROS_INFO("Back to ready position");
		}
		else if (strncmp(cstr, "ready", 5) == 0) {
			double blend = 0;
            std::vector<double> vec1 = {0.0,    -0.4777,    1.9319,    -1.4537,    1.5708,    0.0};
            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            ROS_INFO("Back to ready position");
		}
		else if (strncmp(cstr, "image1", 6) == 0) {
			double blend = 0;
            std::vector<double> vec1 = {0.5508,    -0.576,    1.9309,    0.2183,    1.5586,    0.5456};
            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            ROS_INFO("Back to ready position");
		}
		else if (strncmp(cstr, "VStart", 6) == 0) {
			std::string cmd = "ContinueVJog()";
			script_srv.request.id = "Vstart";
  			script_srv.request.script = cmd;
  			if (script_client.call(script_srv))                             
			{
				if (script_srv.response.ok) ROS_INFO_STREAM("Velocity control mode ON");
				else ROS_WARN_STREAM("Exit script to robot , but response not yet ok ");
			}
			else
			{
				ROS_ERROR_STREAM("Error turn on velocity mode");
				return 1;
			}    	
		}
		else if (strncmp(cstr, "VStop", 6) == 0) {
			std::string cmd = "StopContinueVmode()";
			script_srv.request.id = "Vstop";
  			script_srv.request.script = cmd;
  			if (script_client.call(script_srv))                             
			{
				if (script_srv.response.ok) ROS_INFO_STREAM("Velocity control mode OFF");
				else ROS_WARN_STREAM("Exit script to robot , but response not yet ok ");
			}
			else
			{
				ROS_ERROR_STREAM("Error turn off velocity mode");
				return 1;
			} 
		}
		else if (strncmp(cstr, "VMove", 6) == 0) {
			ROS_INFO("Run velocity control");
			std::vector<double> vec1 = {0.0087,    0.0087,    0.0087,    0.0087,    0.0087,    0.0087};
			vel_srv.request.motion_type = 1;
			vel_srv.request.velocity = vec1;
			vel_client.call(vel_srv);
		}
		else {
			std::string cmd{ cstr };
			std::cout << "send cmd: " << cmd << "\n";

			script_srv.request.id = std::to_string(id_cnt);
  			script_srv.request.script = cmd;
  			if (script_client.call(script_srv))                             
			{
				if (script_srv.response.ok) ROS_INFO_STREAM("Exit script to robot");
				else ROS_WARN_STREAM("Exit script to robot , but response not yet ok ");
			}
			else
			{
				ROS_ERROR_STREAM("Error Exit script to robot");
				return 1;
			}     

		}

		++id_cnt;
		if (id_cnt > 9) { id_cnt = 9; }
	}



	return 0;
}
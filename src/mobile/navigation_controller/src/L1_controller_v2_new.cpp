/*
Copyright (c) 2017, ChanYuan KUO, YoRu LU,
latest editor: HaoChih, LIN
All rights reserved. (Hypha ROS Workshop)

This file is part of hypha_racecar package.

hypha_racecar is free software: you can redistribute it and/or modify
it under the terms of the GNU LESSER GENERAL PUBLIC LICENSE as published
by the Free Software Foundation, either version 3 of the License, or
any later version.

hypha_racecar is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU LESSER GENERAL PUBLIC LICENSE for more details.

You should have received a copy of the GNU LESSER GENERAL PUBLIC LICENSE
along with hypha_racecar.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Path.h"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include "navigation_controller/command.h"
#include <move_base_msgs/MoveBaseActionResult.h> // "move_base/result"
#include <tf/transform_listener.h> // "amcl_pose"
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#define PI 3.14159265358979

/********************/
/* CLASS DEFINITION */
/********************/
class L1Controller
{
    public:
        L1Controller();
        void initMarker();
        bool isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose);
        bool isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos);
        double getYawFromPose(const geometry_msgs::Pose& carPose);        
        double getEta(const geometry_msgs::Pose& carPose);
        double getCar2GoalDist();
        double getL1Distance(const double& _Vcmd);
        double getSteeringAngle(double eta);
        double getGasInput(const float& current_v);
        geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose);
	double getStandingAngle(double currentTheta);

    private:
        ros::NodeHandle n_;
        ros::Subscriber odom_sub, path_sub, goal_sub;
        ros::Publisher pub_, marker_pub;
		ros::Publisher result_; //// "move_base/result"
        ros::Publisher odom_initial_pub;
        ros::Timer timer1, timer2;
        tf::TransformListener tf_listener;

        ros::ServiceClient client_;
		navigation_controller::command srv;

        visualization_msgs::Marker points, line_strip, goal_circle;
        geometry_msgs::Twist cmd_vel;
        geometry_msgs::Point odom_goal_pos;
		geometry_msgs::Quaternion odom_goal_ori;

        nav_msgs::Odometry odom;
        nav_msgs::Path map_path, odom_path;
        std_msgs::Bool odom_initial_flag;

        double L, Lfw, Lrv, Vcmd, lfw, lrv, steering, u, v;
        double Gas_gain, baseAngle, Angle_gain, goalRadius, baseSpeed;
        int controller_freq;
        bool foundForwardPt, goal_received, goal_reached, goal_reached_ori, cmd_flag;
		int smooth_counter;

        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void goalReachingCB(const ros::TimerEvent&);
        void controlLoopCB(const ros::TimerEvent&);

	FILE *pFile;

}; // end of class


L1Controller::L1Controller()
{
    //Private parameters handler
    ros::NodeHandle pn("~");

    //Car parameter
    pn.param("L", L, 0.26);
    pn.param("Lrv", Lrv, 10.0);
    pn.param("Vcmd", Vcmd, 0.4);
    pn.param("lfw", lfw, 0.13);
    pn.param("lrv", lrv, 10.0);

    //Controller parameter
    pn.param("controller_freq", controller_freq, 20);
    pn.param("AngleGain", Angle_gain, -1.0);
    pn.param("GasGain", Gas_gain, 1.0);
    pn.param("baseSpeed", baseSpeed, 0.2);
    pn.param("baseAngle", baseAngle, 0.0);

    //Publishers and Subscribers
    odom_sub = n_.subscribe("/odometry/filtered", 1, &L1Controller::odomCB, this);
    path_sub = n_.subscribe("/move_base_node/NavfnROS/plan", 1, &L1Controller::pathCB, this);
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &L1Controller::goalCB, this);
	odom_initial_pub =n_.advertise<std_msgs::Bool>("odom_initial_flag", 5);
    marker_pub = n_.advertise<visualization_msgs::Marker>("car_path", 10);
    pub_ = n_.advertise<geometry_msgs::Twist>("car/cmd_vel", 1);
    result_ = n_.advertise<std_msgs::String>("/result", 10); //// "move_base/result"

	client_ = n_.serviceClient<navigation_controller::command>("pos_cmd");

    //Timer
    timer1 = n_.createTimer(ros::Duration((0.01)), &L1Controller::controlLoopCB, this); // Duration(0.05) -> 20Hz
    timer2 = n_.createTimer(ros::Duration((0.01)), &L1Controller::goalReachingCB, this); // Duration(0.05) -> 20Hz

    //Init variables
    Lfw = goalRadius = getL1Distance(Vcmd);
    Lfw = 0.8;
    goalRadius = 0.1;
    foundForwardPt = false;
    goal_received = false;
    goal_reached = false;
    goal_reached_ori = false;
    cmd_vel.linear.x = 0; // 0 for stop
    cmd_vel.angular.z = 0;
    smooth_counter = 0;
    cmd_flag = true;

    //Show info
    ROS_INFO("[param] baseSpeed: %f", baseSpeed);
    ROS_INFO("[param] baseAngle: %f", baseAngle);
    ROS_INFO("[param] AngleGain: %f", Angle_gain);
    ROS_INFO("[param] Vcmd: %f", Vcmd);
    ROS_INFO("[param] Lfw: %f", Lfw);

    //Visualization Marker Settings
    initMarker();

    pFile = fopen( "/root/catkin_ws_aris/src/tickets.txt","w" );
}



void L1Controller::initMarker()
{
    points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "/base_footprint";
    points.ns = line_strip.ns = goal_circle.ns = "Markers";
    points.action = line_strip.action = goal_circle.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    goal_circle.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle.type = visualization_msgs::Marker::CYLINDER;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    //LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    goal_circle.scale.x = goalRadius;
    goal_circle.scale.y = goalRadius;
    goal_circle.scale.z = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    //goal_circle is yellow
    goal_circle.color.r = 1.0;
    goal_circle.color.g = 1.0;
    goal_circle.color.b = 0.0;
    goal_circle.color.a = 0.5;
}


void L1Controller::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg) // "amcl_pose"
{
    //get robot position
    tf::StampedTransform transform;
    tf::Quaternion q;

    try {
        tf_listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
	q = transform.getRotation(); 
    }
    catch (tf::TransformException ex) {	
	ROS_ERROR("%s",ex.what());
	ros::Duration(1.0).sleep();
    }
    odom = *odomMsg;
    odom.pose.pose.position.x = transform.getOrigin().x();
    odom.pose.pose.position.y = transform.getOrigin().y();
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(q));
    //ROS_INFO("position.x = %.2f",odom.pose.pose.position.x);
    //ROS_INFO("position.y = %.2f",odom.pose.pose.position.y);
}


void L1Controller::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    map_path = *pathMsg;
}


void L1Controller::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    try
    {
        geometry_msgs::PoseStamped odom_goal;
        //tf_listener.transformPose("/odom", ros::Time(0) , *goalMsg, "/map" ,odom_goal);
	odom_goal = *goalMsg;        
	odom_goal_pos = odom_goal.pose.position;
	odom_goal_ori = odom_goal.pose.orientation;
        goal_received = true;
        goal_reached = false;
	goal_reached_ori = false;

        /*Draw Goal on RVIZ*/
        goal_circle.pose = odom_goal.pose;
        marker_pub.publish(goal_circle);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

double L1Controller::getStandingAngle(double currentTheta){

	geometry_msgs::Pose pose;
	pose.orientation = odom_goal_ori;
	
	double goalTheta = getYawFromPose(pose);
	double deltaTheta;

	double kp = 1.0;
	double w = 0.0;

	//angle between -180 ~ 180 degree
	if (currentTheta > PI)
		currentTheta = currentTheta - int((currentTheta + PI) / (2 * PI)) * (2 * PI);
	else
		currentTheta = currentTheta - int((currentTheta - PI) / (2 * PI)) * (2 * PI);

	if (goalTheta > PI)
		goalTheta = goalTheta - int((goalTheta + PI) / (2 * PI)) * (2 * PI);
	else
		goalTheta = goalTheta - int((goalTheta - PI) / (2 * PI)) * (2 * PI);

	deltaTheta = goalTheta - currentTheta;
	//in order not to turn opposite direction
	if (deltaTheta > PI)
		deltaTheta -= 2.0 * PI;
	else if (deltaTheta < -PI)
		deltaTheta += 2.0 * PI;

	w = kp * deltaTheta;
	ROS_INFO("w : %lf",w);
	if (w >= 30.0 / 180.0 * PI)
		w = 30.0 / 180.0 * PI;
	else if (w <= -30.0 / 180.0 * PI)
		w = -30.0 / 180.0 * PI;
	
	ROS_INFO("w = %lf",w);
	if (-0.025 < w && w < 0.025) // 0.017
		w = 0.0;	
	ROS_INFO("w = %lf",w);

	if(odom_goal_pos.z == 1.0)
		w = 0.0;

	return w;	
}

double L1Controller::getYawFromPose(const geometry_msgs::Pose& carPose)
{
    float x = carPose.orientation.x;
    float y = carPose.orientation.y;
    float z = carPose.orientation.z;
    float w = carPose.orientation.w;

    double tmp,yaw;
    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 quaternion(q);
    quaternion.getRPY(tmp,tmp, yaw);

    return yaw;
}

bool L1Controller::isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
{
    float car2wayPt_x = wayPt.x - carPose.position.x;
    float car2wayPt_y = wayPt.y - carPose.position.y;
    double car_theta = getYawFromPose(carPose);

    float car_car2wayPt_x = cos(car_theta)*car2wayPt_x + sin(car_theta)*car2wayPt_y;
    float car_car2wayPt_y = -sin(car_theta)*car2wayPt_x + cos(car_theta)*car2wayPt_y;

    if(car_car2wayPt_x >0) /*is Forward WayPt*/
        return true;
    else
        return true;
}

bool L1Controller::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos)
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx*dx + dy*dy);

    //ROS_INFO("dist = %.2f when Lfw = %.2f",dist,Lfw);

    if(dist < Lfw)
        return false;
    else if(dist >= Lfw)
        return true;
}

geometry_msgs::Point L1Controller::get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;
    double carPose_yaw = getYawFromPose(carPose);
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point odom_car2WayPtVec;
    foundForwardPt = false;

    if(!goal_reached){
        for(int i =0; i< map_path.poses.size(); i++)
        {
            geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];
            geometry_msgs::PoseStamped odom_path_pose;

            try
            {
                //tf_listener.transformPose("/odom", ros::Time(0) , map_path_pose, "/map" ,odom_path_pose);
		odom_path_pose = map_path_pose;                
		geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
                bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt,carPose);

                if(_isForwardWayPt)
                {
                    bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos);
                    if(_isWayPtAwayFromLfwDist)
                    {
                        forwardPt = odom_path_wayPt;
                        foundForwardPt = true;
		        if((i > map_path.poses.size()-5))
			    Lfw = 0.3;
			else
			    Lfw = 0.8;
		        //ROS_INFO("Robot arrived path_wayPt Last no.%lu",map_path.poses.size()-i);
                        break;
                    }
                    if(getCar2GoalDist()<1.0){
		    	forwardPt = odom_goal_pos;
		    	foundForwardPt = true;
		    }
                }
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }
        //ROS_INFO("path_wayPt is %lf %lf at No.%lu",forwardPt.x,forwardPt.y,map_path.poses.size()-i);
    }
    else if(goal_reached) 
    {
        forwardPt = odom_goal_pos;
        foundForwardPt = false;
        //ROS_INFO("goal REACHED!");
    }

    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
    points.points.clear();
    line_strip.points.clear();
    
    if(foundForwardPt && !goal_reached)
    {
        points.points.push_back(carPose_pos);
        points.points.push_back(forwardPt);
        line_strip.points.push_back(carPose_pos);
        line_strip.points.push_back(forwardPt);
    }

    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    
    odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    fprintf(pFile, "%lf %lf %lf %lf\n", forwardPt.x, forwardPt.y, carPose_pos.x, carPose_pos.y);
    return odom_car2WayPtVec;
}


double L1Controller::getEta(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);

    double eta = atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);
    return eta;
}


double L1Controller::getCar2GoalDist()
{
    geometry_msgs::Point car_pose = odom.pose.pose.position;
    double car2goal_x = odom_goal_pos.x - car_pose.x;
    double car2goal_y = odom_goal_pos.y - car_pose.y;

    double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);

    return dist2goal;
}

double L1Controller::getL1Distance(const double& _Vcmd)
{
    double L1 = 0;
    if(_Vcmd < 1.34)
        L1 = 3 / 3.0; //////////////////////// changed it
        //L1 = 0.2;
    else if(_Vcmd > 1.34 && _Vcmd < 5.36)
        L1 = _Vcmd*2.24 / 3.0;
    else
        L1 = 12 / 3.0;
    return L1;
}

double L1Controller::getSteeringAngle(double eta)
{
    double steeringAnge = -atan2((L*sin(eta)),(Lfw/2+lfw*cos(eta)))*(180.0/PI);
    //ROS_INFO("Steering Angle = %.2f", steeringAnge);
    return steeringAnge;
}

double L1Controller::getGasInput(const float& current_v)
{
    double u = (Vcmd - current_v)*Gas_gain;
    //ROS_INFO("velocity = %.2f\tu = %.2f",current_v, u);
    return u;
}


void L1Controller::goalReachingCB(const ros::TimerEvent&)
{

    if(goal_received)
    {
        double car2goal_dist = getCar2GoalDist();
        //ROS_INFO("car2goal_dist = %.2f when goalRadius = %.2f",car2goal_dist, goalRadius);
	if(odom_goal_pos.z == 1.0){        
		if(car2goal_dist < Lfw/1.5)
			goal_reached = true;
	}	
	else if((car2goal_dist < goalRadius)&&(goal_reached!=true)){
            goal_reached = true;
	    	smooth_counter = 20;
	}
	if(goal_reached == true && goal_reached_ori == true){
		
		if(odom_goal_pos.z == 2.0){
			cmd_flag=false;
            odom_initial_flag.data=true;
            sleep(1);
            odom_initial_pub.publish(odom_initial_flag);
            sleep(2);

            if(odom_goal_pos.x >= 3.0){
                srv.request.type = 2;
			    srv.request.x = 0.4;
			    srv.request.y = 0;
			    srv.request.theta = -1.57;

			    if(client_.call(srv)) {
				    ROS_ERROR("call service success");
				    srv.request.type = 0;
				    while(srv.response.run_completed == false) {
					    usleep(100000);
					    ROS_ERROR("hello servie");
					    client_.call(srv);
				    }
			    }
			    else {
				    ROS_INFO("call service fail");
			    }

                srv.request.type = 1;
                if(client_.call(srv)) {
					ROS_ERROR("call service success");
					srv.request.type = 0;
					while(srv.response.run_completed == false) {
						usleep(100000);
						ROS_ERROR("hello servie");
						client_.call(srv);
					}
				}
				else {
					ROS_INFO("call service fail");
				}
			    sleep(3);
			}else if(odom_goal_pos.x <=3.0){
                srv.request.type = 2;
			    srv.request.x = 0.5;
			    srv.request.y = 0;
			    srv.request.theta = -1.57;

			    if(client_.call(srv)) {
				    ROS_ERROR("call service success");
				    srv.request.type = 0;
				    while(srv.response.run_completed == false) {
					    usleep(100000);
					    ROS_ERROR("hello servie");
					    client_.call(srv);
				    }
			    }
			    else {
				    ROS_INFO("call service fail");
			    }

                srv.request.type = 1;
                if(client_.call(srv)) {
					ROS_ERROR("call service success");
					srv.request.type = 0;
					while(srv.response.run_completed == false) {
						usleep(100000);
						ROS_ERROR("hello servie");
						client_.call(srv);
					}
				}
				else {
					ROS_INFO("call service fail");
				}
			    sleep(3);
			}   
		}


	    goal_reached = false;
	    goal_reached_ori = false;
        goal_received = false;
		cmd_flag=true;
        ROS_INFO("Goal Reached !");
		//fclose(pFile);
	    
	    //// "move_base/result"
	    std_msgs::String goalresult;
	    //goalresult.header.stamp = ros::Time::now();
	    //goalresult.header.frame_id = "/map";
	    goalresult.data = "Reach";
	    result_.publish(goalresult);
        }
    }
}

void L1Controller::controlLoopCB(const ros::TimerEvent&)
{

    geometry_msgs::Pose carPose = odom.pose.pose;
    geometry_msgs::Twist carVel = odom.twist.twist;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;

    if(goal_received)
    {
        /*Estimate Steering Angle*/
        double eta = getEta(carPose);  
        if(!goal_reached)
        {
            cmd_vel.angular.z = baseAngle + getSteeringAngle(eta)*Angle_gain *PI/180.0;
	    
            /*Estimate Gas Input*/
            
                //double u = getGasInput(carVel.linear.x);
                //cmd_vel.linear.x = baseSpeed - u;
                cmd_vel.linear.x = baseSpeed;
		
		if(odom_goal_pos.z == 0.0){		
			if(getCar2GoalDist()<0.5)
				cmd_vel.linear.x = baseSpeed * getCar2GoalDist()/0.5;
		}
                //ROS_INFO("\nGas = %.2f\nSteering angle = %.2f",cmd_vel.linear.x,cmd_vel.angular.z);
		
		if(cmd_vel.angular.z >= 30.0 / 180.0 * PI){
			cmd_vel.angular.z = 30.0 / 180.0 * PI;
			cmd_vel.linear.x = 0;
	    	}
	    	else if(cmd_vel.angular.z <= -30.0 / 180.0 * PI){
			cmd_vel.angular.z = -30.0 / 180.0 * PI;
			cmd_vel.linear.x = 0;
            	}
	  
        }
	else if(!goal_reached_ori){
		if(smooth_counter > 0){       
			smooth_counter--;
                        printf("smmoth %d",smooth_counter);
		}
		else{		
			ROS_INFO("CIRCLEEEEEEEEEEEEEE");
			cmd_vel.angular.z = getStandingAngle(getYawFromPose(carPose));
			if(cmd_vel.angular.z == 0.0)
				goal_reached_ori = true;
		}
	}
	else
	        //cmd_vel.linear.x = 0.1;
		ROS_INFO("Gas = XXX	Steering angle = XXX");
    }
 
    if(odom_goal_pos.z == 1.0)
	cmd_vel.linear.x = baseSpeed*0.8;	

    if(cmd_flag)
    	pub_.publish(cmd_vel);
}


/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "L1Controller_v2");
    L1Controller controller;
    ros::spin();
    return 0;
}

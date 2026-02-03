#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>  // 用於接收人員像素面積

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "navigation_controller/command.h"
#include <string>
#include <sstream>
#include <iostream>
#include <fstream> 

template <typename T>
std::string ToString(T val)
{
    std::stringstream stream;
    stream << val;
    return stream.str();
}

class NavigationController {
public:
	// NavigationController(tf::TransformListener& tf);
	// ~NavigationController();
	// bool set_command(navigation_controller::command::Request &req,navigation_controller::command::Response &res);
	// void run();
	NavigationController(tf::TransformListener &tf) : tf_(tf), human_area_(0.0) {
        ros::NodeHandle nh;
        human_area_sub_ = nh.subscribe("/human_detection_area", 10, &NavigationController::humanAreaCallback, this);
    }

    void humanAreaCallback(const std_msgs::Float32::ConstPtr &msg) {
        human_area_ = msg->data;  // 更新人員像素面積
    }

    void adjustSpeed(geometry_msgs::Twist &cmd_vel) {
        if (human_area_ < AREA_FAR) {
            cmd_vel.linear.x *= 1.0;  // 遠距離：維持原速度
        } 
        else if (human_area_ < AREA_MEDIUM) {
            cmd_vel.linear.x *= 0.5;  // 中距離：減速
        } 
        else {
            cmd_vel.linear.x = 0.0;  // 近距離：立即停止
        }
    }

    void run() {
        ros::NodeHandle nh;
        ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        ros::Rate rate(10);

        while (ros::ok()) {
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0.5;  // 預設速度

            adjustSpeed(cmd_vel);  // 根據影像像素面積來調整速度

            cmd_pub.publish(cmd_vel);
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
	ros::NodeHandle nh_;
	ros::Publisher vel_pub_;
	ros::Subscriber scan_sub_;
	ros::Subscriber poscmd_sub_;
	ros::Publisher pos_status_pub;
	ros::Publisher offset_pub;
	ros::Publisher nav_status_pub;

	ros::Subscriber human_area_sub_;
    float human_area_;  // 記錄影像中的人員像素面積
    const float AREA_FAR = 500.0;  // 遠距離閾值（假設像素面積小於 500 算遠距離）
    const float AREA_MEDIUM = 2000.0;  // 中距離閾值（500 ~ 2000 算中距離）


	ros::Subscriber uwb_sub;
	float uwb_x, uwb_y;
	// geometry_msgs::PointStamped uwb_info;

	tf::TransformListener& tf_;	
	sensor_msgs::LaserScan scan_;
	visualization_msgs::MarkerArray mark_array_;

	ros::Publisher pub_pose;
	geometry_msgs::Pose position;

    //列舉中的每個名稱被指派整數值，這個值對應於它在列舉值順序中的位置。 根據預設，第一個指派的值是 0，下一個指派的是 1，依此類推，不過，您可以明確設定列舉程式的值，如下所示：
	enum NavigationState {
		CONTROLLING,
		APPROACHING,
		FINISH
	};
	NavigationState state_;
	unsigned int command_type_;
	geometry_msgs::PoseStamped goal_;
	geometry_msgs::Twist last_cmd_;
		
	void get_scan(const sensor_msgs::LaserScan::ConstPtr& scan);
	void get_pos(const std_msgs::Float64MultiArray::ConstPtr& array);

	void get_uwb(const geometry_msgs::PointStamped::ConstPtr& msg);

	void angle_control(double *cmd_w, double des_theta, double current_theta);
	void position_control(double *cmd_v, double *cmd_w, double desX, double desY, double currentX, double currentY, double current_theta);


	static const int beam_num;
	double kAvoidRadius_;

	static const double D2R;
	static const double R2D;
	static const double MaxRobotLinearVelocity;
	static const double MaxRobotAngularVelocity;

	bool switch_avoid;
	
};

const double NavigationController::D2R = M_PI / 180.0;			// degrees to rad
const double NavigationController::R2D = 180.0 / M_PI;			// rad to degrees
const double NavigationController::MaxRobotLinearVelocity = 0.4; //0.5	//0.6	// m/s
const double NavigationController::MaxRobotAngularVelocity = 40.0 * D2R; //40 //60	// radian/s
const int NavigationController::beam_num = 30;

NavigationController::NavigationController(tf::TransformListener& tf) : tf_(tf)
{
	//pub the calculated speed into cmd_vel
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	//this let c++ knows that this callback function is in this class
	scan_sub_ = nh_.subscribe("scan", 1, &NavigationController::get_scan, this);
	poscmd_sub_ = nh_.subscribe("pos_cmd", 1, &NavigationController::get_pos, this);

	pos_status_pub = nh_.advertise<std_msgs::String>("pos_status", 1);
	offset_pub = nh_.advertise<std_msgs::String>("offset", 1);
    //pub current status into nav_status (true/ false)
	nav_status_pub = nh_.advertise<std_msgs::String>("nav_status", 1);
	pub_pose = nh_.advertise<geometry_msgs::Pose>("robot_0/pose", 1);

	// uwb_sub = nh_.subscribe("uwb_data", 1000, &NavigationController::get_uwb, this);


	kAvoidRadius_ = 1.0;	//m
	command_type_ = 0;
	state_ = FINISH;
	
	ros::NodeHandle para_node("~");

	// avoidance switch (setting true)
    // get the value into switch_avoid and return true/false for param exist or wrong type
	if(!para_node.getParam("AvoidSwitch", switch_avoid)){
		switch_avoid = true;
	}
	if(switch_avoid == false)
		kAvoidRadius_ = 0.0;

	ROS_INFO_STREAM("contrust the navigation control class");
}

NavigationController::~NavigationController()
{
	ROS_INFO_STREAM("delete the navigation controller class");
}

// void NavigationController::get_uwb(const geometry_msgs::PointStamped::ConstPtr& msg)
// {
// 	// the position of uwb_info are after transfered
// 	uwb_info.point.x = ((msg->point.x) - 60)/100;
// 	uwb_info.point.y = (-(msg->point.y) + 462)/100;
// 	uwb_info.header.frame_id = msg->header.frame_id;
// 	ROS_INFO_STREAM("uwb X = " << uwb_info.point.x << " , uwb Y = " << uwb_info.point.y);
// 	ROS_INFO_STREAM("uwb ID = " << uwb_info.header.frame_id);
// 	// std::cout<< msg<< std::endl;
// }

void NavigationController::get_pos(const std_msgs::Float64MultiArray::ConstPtr& array)
{
	if (array->data.size() == 2) {
		goal_.header.stamp = ros::Time::now();
		goal_.pose.position.x = array->data[0];
		goal_.pose.position.y = array->data[1];
		goal_.pose.orientation.z = 0.0;
		state_ = CONTROLLING;
		command_type_ = 2;
		ROS_INFO_STREAM("Control postition, X = " << goal_.pose.position.x << " Y = " << goal_.pose.position.y );
	}
	else if(array->data.size() == 1) {
		goal_.header.stamp = ros::Time::now();
		goal_.pose.position.x = 0.0;
		goal_.pose.position.y = 0.0;
		goal_.pose.orientation.z = array->data[0];
		state_ = CONTROLLING;
		command_type_ = 1;
		ROS_INFO_STREAM("Control orientation, theta = " << goal_.pose.orientation.z );
	}
	
}
// get position command, set goal pose , state, command_type
bool NavigationController::set_command(navigation_controller::command::Request &req,navigation_controller::command::Response &res)
{
    // get goal lacation
	if (req.type == 2) {
        //goal_  is type of geometry_msgs::PoseStamped
		goal_.header.stamp = ros::Time::now();
		goal_.pose.position.x = req.x;
		goal_.pose.position.y = req.y;
		goal_.pose.orientation.z = 0.0;
		state_ = CONTROLLING;
		command_type_ = req.type;
		ROS_INFO_STREAM("Control postition, X = " << goal_.pose.position.x << " Y = " << goal_.pose.position.y );
	}
	else if(req.type == 1) {
		goal_.header.stamp = ros::Time::now();
		goal_.pose.position.x = 0.0;
		goal_.pose.position.y = 0.0;
		goal_.pose.orientation.z = req.theta;
		state_ = CONTROLLING;
		command_type_ = req.type;
		ROS_INFO_STREAM("Control orientation, theta = " << goal_.pose.orientation.z );
	}
	else if(req.type == 0) {
		if(state_ == CONTROLLING) {
			res.run_completed = false;
		}
		else {
			res.run_completed = true;
		}
	}
	return true;
}


void NavigationController::run()
{
	ros::Rate rate(10);

	// Status::Topic
	std_msgs::String pos_status;
	// open file as output(writting data)
	std::ofstream out_0("/home/tm5/Documents/mobile/src/mobile/navigation_controller/cmd_txt/cmd_history.txt");
	//total command
	double v_t = 0.0;
	double w_t = 0.0;

	//navigation command
	double v_n = 0.0;
	double w_n = 0.0;

	double Ea = 0.0;	//confidence factor of avoidance


	geometry_msgs::Twist cmd;
	while (ros::ok()) {

		//get robot position
		tf::StampedTransform transform;
		tf::Quaternion q;
		try {
			//用於接收最新的transform, 4個輸入:1＆2. 想要1到2的轉換 3.當下時間 4.用於儲存的object 
			//ros::Time(0)為緩衝區中最新可用的轉換
			tf_.lookupTransform("/map", "/base_link", ros::Time(0), transform);
			// tf_.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
			q = transform.getRotation(); 
				// std::cout << q.x() << " ," << q.y() << " ,"
			//               << q.z() << " ," << q.w() << std::endl;
		}
		//find out what err
		catch (tf::TransformException ex) {	
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			ROS_ERROR("Transform err !!!!!!!!!!!!!!!!!!!!!!");
		}

		position.position.x=transform.getOrigin().x();
    	position.position.y=transform.getOrigin().y();
    	position.position.z=tf::getYaw(q);
    	position.orientation=tf::createQuaternionMsgFromYaw(tf::getYaw(q));
    	pub_pose.publish(position);


		// Status:: pos_status topic
		pos_status.data = "[pos_x : " + ToString(transform.getOrigin().x())+", pos_y : "+ToString(transform.getOrigin().y())+", theta : "+ToString(tf::getYaw(q))+" ]";
		pos_status_pub.publish(pos_status);

		if(state_ == CONTROLLING || state_ == APPROACHING) {
			v_t = 0.0;
			w_t = 0.0;
			v_n = 0.0;
			w_n = 0.0;

			//ROS_INFO_STREAM("x = " << transform.getOrigin().x() * 100 << " y = " << transform.getOrigin().y() * 100 << " theta = " << tf::getYaw(q) * R2D);

			//calculate navigation v & w (w is rotarion speed)
			if(command_type_ == 2) {
				//calculate the linear velocity by goal position and current position, and set some limits(extreme value ,oscilliate)
				position_control(&v_n,&w_n,goal_.pose.position.x,goal_.pose.position.y,transform.getOrigin().x(),transform.getOrigin().y(),tf::getYaw(q));
				if(v_n == 0.0 && w_n == 0.0)
					state_ = FINISH;
				else if(sqrt(pow(goal_.pose.position.x-transform.getOrigin().x(),2)+pow(goal_.pose.position.y-transform.getOrigin().y(),2)) < 0.0)
					state_ = APPROACHING;
			}
			else if(command_type_ == 1) {
				//calculate the angular velocity
				angle_control(&w_n,goal_.pose.orientation.z,tf::getYaw(q));
				if(w_n == 0.0)
					state_ = FINISH;
				else if(fabs(goal_.pose.orientation.z-tf::getYaw(q)) < 0.0 * D2R)
					state_ = APPROACHING;
			}
	
			out_0<<std::to_string(v_n)<<" "<<std::to_string(w_n)<<std::endl;
			v_t = v_n;
			w_t = w_n;
		}

		double acc_v = 0.05;
		double acc_w = 10 * D2R;
		//avoid velocity change too fast
		//only use linear.x and angulat.z
		if(fabs(v_t - last_cmd_.linear.x) < acc_v)
			cmd.linear.x = v_t;
		else if((v_t - last_cmd_.linear.x) > 0)
			cmd.linear.x = last_cmd_.linear.x + acc_v;
		else if((v_t - last_cmd_.linear.x) < 0)
			cmd.linear.x = last_cmd_.linear.x - acc_v;
		
		//avoid angular speed change too fast
		if(fabs(w_t - last_cmd_.angular.z) < acc_w)
			cmd.angular.z = w_t;
		else if((w_t - last_cmd_.angular.z) > 0)
			cmd.angular.z = last_cmd_.angular.z + acc_w;
		else if((w_t - last_cmd_.angular.z) < 0)
			cmd.angular.z = last_cmd_.angular.z - acc_w;
		
		vel_pub_.publish(cmd);
		last_cmd_ = cmd;

		// Status::Topic
		std_msgs::String nav_status;
		// Status:: nav_status topic
		if(state_ != FINISH )
			nav_status.data = "true";
		else
			nav_status.data = "false";
		nav_status_pub.publish(nav_status);


		ros::spinOnce();
		rate.sleep();
	}
}

void NavigationController::get_scan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	scan_ = *scan;
}

//(rad/s ,rad ,rad)
void NavigationController::angle_control(double *cmdW, double desTheta, double currentTheta)
{
	// Status::Topic
	std_msgs::String offset;
	
	double deltaTheta = 0.0;
	double kp = 1.5;
	double w = 0.0;

	//angle between -180 ~ 180 degree
	if (desTheta > M_PI)
		desTheta = desTheta - int((desTheta + M_PI) / (2 * M_PI)) * (2 * M_PI);
	else
		desTheta = desTheta - int((desTheta - M_PI) / (2 * M_PI)) * (2 * M_PI);

	//angle between -180 ~ 180 degree
	if (currentTheta > M_PI)
		currentTheta = currentTheta - int((desTheta + M_PI) / (2 * M_PI)) * (2 * M_PI);
	else
		currentTheta = currentTheta - int((desTheta - M_PI) / (2 * M_PI)) * (2 * M_PI);
	
	deltaTheta = desTheta - currentTheta;
	//in order not to turn opposite direction
	if (deltaTheta > M_PI)
		deltaTheta -= 2.0 * M_PI;
	else if (deltaTheta < -M_PI)
		deltaTheta += 2.0 * M_PI;

	// Status:: offset topic
	offset.data = "deltaX : --; deltaY : --; deltaTheta : "+ToString(deltaTheta*180.0);
	offset_pub.publish(offset);

	w = kp * deltaTheta;

	if (fabs(w) > fabs(MaxRobotAngularVelocity))
		w = w / fabs(w) * fabs(MaxRobotAngularVelocity);
	else
		w = w / fabs(w) * std::max(5.0 * D2R, fabs(w));

	if (fabs(deltaTheta) <= 1.0 * D2R)
		w = 0.0;

	//return command
	*cmdW = w;
}

//(m/s ,rad/s ,m ,m ,m ,m ,rad)
void NavigationController::position_control(double *cmdV, double *cmdW, double desX, double desY, double currentX, double currentY, double currentTheta)
{
	// Status::Topic
	std_msgs::String offset;
	
	double kp = 2.0;		//kp of rotation 
	double v = 0.0;
	double w = 0.0;

	double deltaX = 0.0;		//m
	double deltaY = 0.0;		//m
	double deltaR = 0.0;		//m
	double angle = 0.0;		//radian
	double deltaTheta = 0.0;	//radian


	deltaX = desX - currentX;
	deltaY = desY - currentY;
	 
	// Status:: offset topic
	offset.data = "deltaX : "+ToString(deltaX)+"; deltaY : "+ToString(deltaY)+"; deltaTheta : --";
	offset_pub.publish(offset);

	deltaR = sqrt(pow(deltaX, 2) + pow(deltaY, 2));	
	//in radian between -pi ~ pi
	angle = atan2(deltaY, deltaX);	
	//angle between -180 ~ 180 degree  M_pi == pi
	if (currentTheta > M_PI)
		currentTheta = currentTheta - int((currentTheta + M_PI) / (2 * M_PI)) * (2 * M_PI);
	else
		currentTheta = currentTheta - int((currentTheta - M_PI) / (2 * M_PI)) * (2 * M_PI);

	deltaTheta = angle - currentTheta;
	//in order not to turn opposite direction
	if (deltaTheta > M_PI)
		deltaTheta -= 2.0 * M_PI;
	else if (deltaTheta < -M_PI)
		deltaTheta += 2.0 * M_PI;

	w = kp * deltaTheta;
	if (w == 0.0)	//not to divide by zero
		;
	else if (fabs(w) > MaxRobotAngularVelocity)  //fabs(x) return absolute value of x 
		w = w / fabs(w) * MaxRobotAngularVelocity;
	else
		w = w / fabs(w) * std::max(5.0 * D2R, fabs(w));	//adjustment for both of wheel speed not too low

	if (deltaR == 0.0)	//not to divide by zero
		;
	else if (fabs(deltaR) > fabs(MaxRobotLinearVelocity * 1.0))
		v = deltaR / fabs(deltaR) * fabs(MaxRobotLinearVelocity);
	else
		v = deltaR / fabs(deltaR) * std::max(0.10, fabs(deltaR / 2.0));	//low speed maybe cause robot can not move

	//reach the goal
	if (fabs(deltaTheta) <= 0.5 * D2R || fabs(deltaR) <= 0.025) // default = 1.0 * D2R || 0.2
		w = 0.0;
	if (fabs(deltaR) <= 0.025)	// default = 0.2
		v = 0.0;

	//return command
	*cmdV = v;
	*cmdW = w;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "navigation_controller");
	ros::NodeHandle nh;
	tf::TransformListener tf(ros::Duration(10));
	NavigationController navigation_controller(tf);
	//pos_cmd use for giving point //the last input is for callback function using class's variant
	ros::ServiceServer service = nh.advertiseService("pos_cmd", &NavigationController::set_command, &navigation_controller);

	navigation_controller.run();

	ros::Rate rate(10);
	while (ros::ok()) {

		ros::spinOnce();
		rate.sleep();
	}
	ros::spin();
	return 0;
}

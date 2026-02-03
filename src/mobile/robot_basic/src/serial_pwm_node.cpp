#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <robot_basic_msgs/Data.h>
#include <math.h>

serial::Serial ser;

std::string port;
int baudrate;
double Arr[90];
double MotorSpeed[4]={0.0,0.0,0.0,0.0};

void initial();
void write_serial(const std_msgs::Float64MultiArray::ConstPtr& pwm);
char str[10];
int main (int argc, char** argv)
{
	ros::init(argc, argv, "serial_pwm_node");
	ros::NodeHandle nh;
	ros::Subscriber motor_pwm_sub = nh.subscribe("motor_pwm", 10, write_serial);
	ros::Publisher motor_cmd_pub = nh.advertise<robot_basic_msgs::Data>("arduino_pwm", 10);

	ros::Rate loop_rate(20);
	ros::Time current_time;
	int Motor_cmd[2]={0,0};
        std_msgs::Int32 motor_pwm;

        robot_basic_msgs::Data cmd;
	while(ros::ok()) {

		//printf("%lf:%lf:%lf:%lf:\n", MotorSpeed[0], MotorSpeed[1], MotorSpeed[2], MotorSpeed[3]);
		motor_pwm.data=(int)(MotorSpeed[0]/1.75*255);
		cmd.motor_pwm_l=motor_pwm;
		motor_pwm.data=(int)(MotorSpeed[1]);
		cmd.motor_dir_l=motor_pwm;
		motor_pwm.data=(int)(MotorSpeed[2]/1.75*255);
		cmd.motor_pwm_r=motor_pwm;
		motor_pwm.data=(int)(MotorSpeed[3]);
		cmd.motor_dir_r=motor_pwm;


		motor_cmd_pub.publish(cmd);
		ros::spinOnce();
		loop_rate.sleep();
	}

}

void initial()
{
	ros::NodeHandle para_node("~");
	if(!para_node.getParam("port", port))
    		port = "/dev/ttyACM0";
	if(!para_node.getParam("baudrate", baudrate))
    		baudrate = 57600;
}
void write_serial(const std_msgs::Float64MultiArray::ConstPtr& pwm)
{
	int i = 0;
	// print all the remaining numbers
	for(std::vector<double>::const_iterator it = pwm->data.begin(); it != pwm->data.end(); ++it)
	{
		Arr[i] = *it;
		i++;
	}
        MotorSpeed[0]=Arr[0];
	MotorSpeed[1]=Arr[1];
	MotorSpeed[2]=Arr[2];
	MotorSpeed[3]=Arr[3];

}

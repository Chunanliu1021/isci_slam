#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <math.h>

const double WheelDistance = 0.56;               //m
double desire_velocity[2]={0.0,0.0};

void write_serial(const geometry_msgs::Twist::ConstPtr& speed);

int main (int argc, char** argv)
{

	ros::init(argc, argv, "serial_dsp_node");
	ros::NodeHandle nh;

	ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 10, write_serial);
	ros::Publisher motor_cmd_pub = nh.advertise<std_msgs::Float64MultiArray>("motor_pwm", 10);

	ros::Rate loop_rate(20);

	std_msgs::Float64MultiArray motor_pwm;
	motor_pwm.data.clear();

        double leftmotorvalue = desire_velocity[0];
        double reverse_l;
        double rightmotorvalue = desire_velocity[1];
        double reverse_r;
	
	ros::Time current_time;
	while(ros::ok()) {
		ros::spinOnce();
		current_time = ros::Time::now();

		leftmotorvalue = desire_velocity[0];
		rightmotorvalue = desire_velocity[1];
		//printf("Desired rightmotorvalue:%lf  Desired leftmotorvalue:%lf\n",-rightmotorvalue,leftmotorvalue);
		if(leftmotorvalue>0.0){
			reverse_l=1;
		}else{
			reverse_l=0;
			leftmotorvalue=-leftmotorvalue;
		}


		if(rightmotorvalue>0.0){
			reverse_r=0;
		}else{
			reverse_r=1;
			rightmotorvalue=-rightmotorvalue;
		}
                                                                                      
		motor_pwm.data.push_back(leftmotorvalue);
		motor_pwm.data.push_back(reverse_l);
		motor_pwm.data.push_back(rightmotorvalue);
		motor_pwm.data.push_back(reverse_r);
		motor_cmd_pub.publish(motor_pwm);

  		motor_pwm.data.clear();

			
		loop_rate.sleep();
	}
}



void write_serial(const geometry_msgs::Twist::ConstPtr& speed)
{
	double vr = (speed->linear.x * 2.0 + speed->angular.z * WheelDistance) / 2.0;
	double vl = (speed->linear.x * 2.0 - speed->angular.z * WheelDistance) / 2.0;
	desire_velocity[0]=vl;
	desire_velocity[1]=-vr;

}

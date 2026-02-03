#include <ros/ros.h>
#include <serial/serial.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/String.h>
#include <math.h>
#include <vector>
#include <string>
#include <sstream>

using namespace std;

const double WheelDistance = 0.556;		//m
const double WheelRadius = 0.075;		//m
const double EncoderPlus = 2500.0;
const double GearRatio = 20.0;
const double DriverSampleTime = 0.066;		//s
const unsigned long fourthmultiple = pow(2, 24);
const unsigned int thirdmultiple = pow(2, 16);
const unsigned long secondmultiple = pow(2, 8);

//serial parameter
/////////////////////////
serial::Serial serL;
serial::Serial serR;
std::string Lport, Rport;
int baudrate;

bool writing = false;
bool speedset_flag = false;

//Motor Cmd
std_msgs::UInt8MultiArray lmotorcmd;
std_msgs::UInt8MultiArray rmotorcmd;

// Status::Topic
std_msgs::String motor_rotate;
std_msgs::String odometry;
double odometry_value = 0;

void initial();
void write_serial(const geometry_msgs::Twist::ConstPtr& speed);

template <typename T>
std::string ToString(T val)
{
    std::stringstream stream;
    stream << val;
    return stream.str();
}

int main (int argc, char** argv)
{

	ros::init(argc, argv, "serial_driver_node");
	ros::NodeHandle nh;

	ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 5, write_serial);
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);
	ros::Publisher motor_rotate_pub = nh.advertise<std_msgs::String>("motor_rotate", 1);
	ros::Publisher odometry_pub = nh.advertise<std_msgs::String>("odometry", 1);

	tf::TransformBroadcaster odom_broadcaster;

	initial();
	//connect to serial
	try{
/* back
		//Left port
		serL.setPort(Lport);
		serL.setBaudrate(baudrate);
		serial::Timeout Lto = serial::Timeout::simpleTimeout(0);
		serL.setTimeout(Lto);
		serL.open();
*/
		//Right port
		serR.setPort(Rport);
		serR.setBaudrate(baudrate);
		serial::Timeout Rto = serial::Timeout::simpleTimeout(0);
		serR.setTimeout(Rto);
		serR.open();

	}
	catch (serial::IOException& e){
		ROS_ERROR_STREAM("Unable to open port ");
		return -1;
	}

/*  back
	//open or not
	if(serL.isOpen()){
		ROS_INFO_STREAM("Serial Leftwheel Port initialized");
	}
	else{
		return -1;
	}
*/
	if(serR.isOpen()){
		ROS_INFO_STREAM("Serial Rightwheel Port initialized");
	}
	else{
		return -1;
	}

	ros::Rate loop_rate(15);


	std_msgs::UInt8MultiArray array;
	std_msgs::UInt8MultiArray array_;
	std_msgs::UInt8MultiArray arrayR;
	int count[2] = {0, 0};

	//FirstEncoder
	vector<unsigned char> flencoder(4);
	vector<unsigned char> frencoder(4);
	long int fencoderPulse[2] = {0, 0};
	vector<bool> initial(2);  //initial[0] := left
	initial[0] = true;
	initial[1] = true;
	//CurrentEncoder
	vector<unsigned char> lencoder(4);
	vector<unsigned char> rencoder(4);
	long int encoderPulse[2] = {0, 0};

	double Odometry[3] = {0.0};	//x,y,theta (m,m,rad) -pi~pi
	ros::Time current_time;
	while(ros::ok()){
		array.data.clear();
		ROS_ERROR("FUCK1");
		cout << "hello1\n";

 		/*array.data.push_back(0x00);
		array.data.push_back(0x65);
		array.data.push_back(0x02);
		array.data.push_back(0x01);
		array.data.push_back(0x63);
		array.data.push_back(0x00);
		array.data.push_back(0x00);
		array.data.push_back(0x00);
		array.data.push_back(0x00);
		array.data.push_back(0x02);
		array.data.push_back(0x63);
		array.data.push_back(0x00);
		array.data.push_back(0x00);
		array.data.push_back(0x00);
		array.data.push_back(0x00);
		array.data.push_back(0xEB);
		array.data.push_back(0xF4);*/


 		/*array.data.push_back(0x00);
		array.data.push_back(0x65);
		array.data.push_back(0x02);
		array.data.push_back(0x01);
		array.data.push_back(0x0A);
		array.data.push_back(0x00);
		array.data.push_back(0x00);
		array.data.push_back(0x01);
		array.data.push_back(0x2C);
		array.data.push_back(0x02);
		array.data.push_back(0x0A);
		array.data.push_back(0x00);
		array.data.push_back(0x00);
		array.data.push_back(0xFE);
		array.data.push_back(0xD4);
		array.data.push_back(0x0B);
		array.data.push_back(0x51);*/
		
        array.data.push_back(0x00);
		array.data.push_back(0x65);
		array.data.push_back(0x02);
		array.data.push_back(0x01);
		array.data.push_back(0x00);
		array.data.push_back(0x00);
		array.data.push_back(0x00);
		array.data.push_back(0x00);
		array.data.push_back(0x00);
		array.data.push_back(0x02);
		array.data.push_back(0x00);
		array.data.push_back(0x00);
		array.data.push_back(0x00);
		array.data.push_back(0x00);
		array.data.push_back(0x00);
		array.data.push_back(0xDE);
		array.data.push_back(0xB9);
		

 		/*array.data.push_back(0x00);
		array.data.push_back(0x65);
		array.data.push_back(0x02);
		array.data.push_back(0x01);
		array.data.push_back(0x10);
		array.data.push_back(0x01);
		array.data.push_back(0x2C);
		array.data.push_back(0x07);
		array.data.push_back(0xD0);
		array.data.push_back(0x02);
		array.data.push_back(0x10);
		array.data.push_back(0x01);
		array.data.push_back(0x36);
		array.data.push_back(0x05);
		array.data.push_back(0xDC);
		array.data.push_back(0xA5);
		array.data.push_back(0xAE);*/



/*
		array.data.push_back(0x00);
		array.data.push_back(0x65);
		array.data.push_back(0x01);
		array.data.push_back(0x01);
		array.data.push_back(0x63);
		array.data.push_back(0x00);
		array.data.push_back(0x00);
		array.data.push_back(0x00);
		array.data.push_back(0x00);
		array.data.push_back(0x19);
		array.data.push_back(0xFE);*/
/*
		array.data.push_back(0x01);
		array.data.push_back(0x06);
		array.data.push_back(0x3f);
		array.data.push_back(0x08);
		array.data.push_back(0x03);
		array.data.push_back(0xe8);
		array.data.push_back(0x04);
		array.data.push_back(0xa2);*/
		//array.data.push_back(0x00);
		//array.data.push_back(0x18);
		//array.data.push_back(0x2F);
        
        /*array.data.push_back(0x01);
		array.data.push_back(0x06);
		array.data.push_back(0x01);
		array.data.push_back(0x00);
		array.data.push_back(0xff);
		array.data.push_back(0xff);
		array.data.push_back(0x89);
		array.data.push_back(0x86);/*
		ROS_ERROR("FUCK2");
		cout << "hello2\n";
/*
		array.data.push_back(0x01);
		array.data.push_back(0x06);
		array.data.push_back(0x01);
		array.data.push_back(0x00);
		array.data.push_back(0xff);
		array.data.push_back(0xff);
		array.data.push_back(0x89);
		array.data.push_back(0x86);*/
/*
		array.data.push_back(0x00);
		array.data.push_back(0x65);
		array.data.push_back(0x02);
		array.data.push_back(0x01);
		array.data.push_back(0x0A);
		array.data.push_back(0x00);
		array.data.push_back(0x00);
		array.data.push_back(0x01);
		array.data.push_back(0x2C);
		array.data.push_back(0x02);
		array.data.push_back(0x0A);
		array.data.push_back(0x00);
		array.data.push_back(0x00);
		array.data.push_back(0xFE);
		array.data.push_back(0xD4);
		array.data.push_back(0x0B);
		array.data.push_back(0x51);*/
		//array.data.push_back(0x2f);
		//back  serL.write(array.data);
		serR.write(array.data);
		//Receive data have to delay!! very important!!
		ros::Duration(0.05).sleep();
		//LeftEncoderInformation
/*		while(1){
 back
			if(serL.available() >= 8){
				//ROS_INFO_STREAM("Reading from serial port");
				array.data.clear();
				//read 8 unsigned char
				serL.read(array.data,8);
				for(int i = 0; i < 4; i++){
					lencoder[i] = array.data[i+2];
				}
				//std::cout<<"LeftEncoderInfo=" << (int)(unsigned int)array.data[0] <<" ;" <<(int)(unsigned int)array.data[1] <<" ;" <<(int)(unsigned int)array.data[2] <<" ;" << (int)(unsigned int)array.data[3] <<" ;"<<(int)(unsigned int)array.data[4] <<" ;" <<(int)(unsigned int)array.data[5] << "\n";
				if(initial[0] == true){
					//left initial value
					for(int i = 0; i < 4; i++){
						flencoder[i] = lencoder[i];
					}
				}
				break;
			}else{
				count[0]++;
				if(count[0] >= 3){
					count[0] = 0;
					break;
				}
			}

		}
*/
		//RightEncoderInformation
		while(1){
			
			if(serR.available() >= 1){
				ROS_ERROR("FUCK3");	
				cout << "hello3\n";			
				//ROS_INFO_STREAM("Reading from serial port");
				array.data.clear();
				array_.data.clear();
				//read 8 unsigned char
				serR.read(array_.data,8);
				for(int i = 0; i < 4; i++){
					rencoder[i] = array_.data[i+2];
				}
				std::cout<<"RightEncoderInfo=" << (int)(unsigned int)array_.data[0] <<" ;" <<(int)(unsigned int)array_.data[1] <<" ;" <<(int)(unsigned int)array_.data[2] <<" ;" << (int)(unsigned int)array_.data[3] <<" ;"<<(int)(unsigned int)array_.data[4] <<" ;" <<(int)(unsigned int)array_.data[5] <<" ;"<<(int)(unsigned int)array_.data[6] <<" ;"<<(int)(unsigned int)array_.data[7] << "\n";
				ROS_ERROR("X: %d",(int)(unsigned int)array_.data[4]);
				if(initial[1] == true){
					//left initial value
					for(int i = 0; i < 4; i++){
						frencoder[i] = rencoder[i];
					}
				}
				break;
			}else{
				count[1]++;
				if(count[1] >= 3){
					count[1] = 0;
					break;
				}
			}

		}

		//Compute Odometry
		current_time = ros::Time::now();
		//cout << "Multiple:" << fourthmultiple << " ;" << thirdmultiple << " ;" << secondmultiple << "\n";

		//left
		if ((~lencoder[0]+1) < 0){
			encoderPulse[0] = lencoder[0] * fourthmultiple + lencoder[1] * thirdmultiple +lencoder[2] * secondmultiple + lencoder[3] - 4294967296;
		}else{
			encoderPulse[0] = lencoder[0] * fourthmultiple + lencoder[1] * thirdmultiple +lencoder[2] * secondmultiple + lencoder[3];
		}
		//right
		if ((~rencoder[0]+1) < 0){
			encoderPulse[1] = rencoder[0] * fourthmultiple + rencoder[1] * thirdmultiple +rencoder[2] * secondmultiple + rencoder[3] - 4294967296;
		}else{
			encoderPulse[1] = rencoder[0] * fourthmultiple + rencoder[1] * thirdmultiple +rencoder[2] * secondmultiple + rencoder[3];
		}
#if 1
		//initialvalue
		if(initial[0] == true){
			initial[0] = false;
			initial[1] = false;
			fencoderPulse[0] = encoderPulse[0];
			fencoderPulse[1] = encoderPulse[1];
			cout << "hello\n";
		}
		int WPluse[2] = { 0, 0 };
		WPluse[0] = -(encoderPulse[0] - fencoderPulse[0]);
		WPluse[1] = (encoderPulse[1] - fencoderPulse[1]);
		fencoderPulse[0] = encoderPulse[0];
		fencoderPulse[1] = encoderPulse[1];
#endif
		//cout << "LencoderPulse : " << encoderPulse[0] << ", RencoderPulse : " << encoderPulse[1] << "\n\n";

		//Odometry
		//cout << "Odometry : \n";

		double WV[2] = { 0.0, 0.0 };
		WV[0] = ((double)WPluse[0] / EncoderPlus) * 0.03735 * (2.0 * M_PI * WheelRadius) / GearRatio / DriverSampleTime;
		WV[1] = ((double)WPluse[1] / EncoderPlus) * 0.03735 * (2.0 * M_PI * WheelRadius) / GearRatio / DriverSampleTime;

		double LinearVelocity = (WV[1] + WV[0]) / 2.0;
		double AngularVelocity = -(WV[1] - WV[0]) / WheelDistance;

		odometry_value = odometry_value + std::abs(LinearVelocity * DriverSampleTime);

		double deltatheta = -(WV[1] - WV[0]) * DriverSampleTime / WheelDistance;
		double deltas = (WV[1] + WV[0]) * DriverSampleTime / 2.0;

		double thetak = Odometry[2];	//theta of last time
		Odometry[2] = (Odometry[2] + deltatheta) ;
		Odometry[0] = Odometry[0] + deltas * cos((Odometry[2] + thetak) / 2.0) ;
		Odometry[1] = Odometry[1] + deltas * sin((Odometry[2] + thetak) / 2.0);
		if (Odometry[2] > M_PI)
			Odometry[2] -= (2.0 * M_PI);
		else if (Odometry[2] < -M_PI)
			Odometry[2] += (2.0 * M_PI);

		//ROS_INFO_STREAM("odom x: " << Odometry[0] << " odom y: " << Odometry[1] << " odom theta: " << Odometry[2] * 180.0 / M_PI);
		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Odometry[2]);


		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = Odometry[0];
		odom_trans.transform.translation.y = Odometry[1];
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);


		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = Odometry[0];
		odom.pose.pose.position.y = Odometry[1];
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = LinearVelocity;
		odom.twist.twist.linear.y = 0.0;
		odom.twist.twist.angular.z = AngularVelocity;

		//publish the message
		odom_pub.publish(odom);

		// Status:: motor_rotate topic
		motor_rotate.data = "LinearVelocity : "+ ToString(LinearVelocity)+"; AngularVelocity : "+ ToString(AngularVelocity); 
		motor_rotate_pub.publish(motor_rotate);

		// Status:: odometry topic
		odometry.data = "Odometry : "+ ToString(odometry_value);
		odometry_pub.publish(odometry);
		ROS_ERROR("FUCK4");
		cout << "hello4\n";
		loop_rate.sleep();

		ros::spinOnce();

	}
	//back  serL.close();
	serR.close();
}


void initial(){
	ros::NodeHandle para_node("~");
	if(!para_node.getParam("Lport", Lport))
    		Lport = "/dev/ttyUSB1";
	if(!para_node.getParam("baudrate", baudrate))
    		baudrate = 115200;
  	if(!para_node.getParam("Rport", Rport))
    		Rport = "/dev/ttyUSB0";
}
void write_serial(const geometry_msgs::Twist::ConstPtr& speed)
{
		
	//unsigned int MotorCmd[11] = { 0x00, 0x65, 0x01, 0x01, 0x0A, 0x00 ,0x00,0x01,0x2c,0x45,0xBA};
	//unsigned int MotorCmd[11] = { 0x00, 0x65, 0x01, 0x01, 0x10, 0x01 ,0x2c,0x07,0xd0,0xdf,0xac};
	unsigned int MotorCmd[8] = { 0x00, 0x06, 0x3f, 0x08, 0x03, 0xe8 ,0x05,0x73};	
	//char r_hex[4];
	//char l_hex[4];

	double vr = (speed->linear.x * 2.0 + speed->angular.z * WheelDistance) / 2.0;
	double vl = (speed->linear.x * 2.0 - speed->angular.z * WheelDistance) / 2.0;
	int leftmotorvalue = (int)(round(vl / (2.0 * M_PI *  WheelRadius) * 60.0));
	int rightmotorvalue = (int)(round(vr / (2.0 * M_PI *  WheelRadius) * 60.0));
	if (leftmotorvalue > 250)
		leftmotorvalue = 250;
	else if (leftmotorvalue < -250)
		leftmotorvalue = -250;

	if (rightmotorvalue > 250)
		rightmotorvalue = 250;
	else if (rightmotorvalue < -250)
		rightmotorvalue = -250;
	//int motorLcmd = (int)(round(leftmotorvalue * 16384.0 / 6000.0));
	//int motorRcmd = (int)(round(rightmotorvalue * 16384.0 / 6000.0));
	int motorLcmd = (int)(round(leftmotorvalue * 25));
	int motorRcmd = (int)(round(rightmotorvalue * 25));
	ROS_ERROR("FUCK");
	
	//std::cout<<"RMotorRPM = "<< rightmotorvalue <<" ;LMotorRPM = "<< leftmotorvalue << " ;" << " ;motorRcmd = "<< motorRcmd << " ;motorLcmd = "<< motorLcmd << "\n";
	//std::cout<<"RMotorRPM = "<< rightmotorvalue <<" ;LMotorRPM = "<< leftmotorvalue <<"\n";
	//MotorCmd[3] = motorRcmd&0XFF;	//low 8
  	//MotorCmd[4] = motorRcmd>>8;	//high 8
	//MotorCmd[5] = motorLcmd&0XFF;	//low 8
  	//MotorCmd[6] = motorLcmd>>8;	//high 8
	//MotorCmd[8]=MotorCmd[0]^MotorCmd[1]^MotorCmd[2]^MotorCmd[3]^MotorCmd[4]^MotorCmd[5]^MotorCmd[6]^MotorCmd[7];
	lmotorcmd.data.clear();
	lmotorcmd.data.push_back(MotorCmd[0]);
	lmotorcmd.data.push_back(MotorCmd[1]);
	lmotorcmd.data.push_back(MotorCmd[2]);
	lmotorcmd.data.push_back(MotorCmd[3]);
	lmotorcmd.data.push_back(MotorCmd[4]);
	lmotorcmd.data.push_back(MotorCmd[5]);
	lmotorcmd.data.push_back(MotorCmd[6]);
	lmotorcmd.data.push_back(MotorCmd[7]);
	//lmotorcmd.data.push_back(MotorCmd[8]);
	//lmotorcmd.data.push_back(MotorCmd[9]);
	//lmotorcmd.data.push_back(MotorCmd[10]);
	//lmotorcmd.data.push_back(MotorCmd[8]);
	//back  serL.write(lmotorcmd.data);0
	serR.write(lmotorcmd.data);


}


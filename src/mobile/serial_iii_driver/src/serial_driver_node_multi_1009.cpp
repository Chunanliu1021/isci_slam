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

const double WheelDistance = 0.45;		//m
const double WheelRadius = 0.075;		//m
const double EncoderPlus = 2500.0;
const double GearRatio = 30.0;
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
unsigned int calc_crc(unsigned int *buf, int length);

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

	ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 1, write_serial);
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);
	ros::Publisher motor_rotate_pub = nh.advertise<std_msgs::String>("motor_rotate", 1);
	ros::Publisher odometry_pub = nh.advertise<std_msgs::String>("odometry", 1);

	tf::TransformBroadcaster odom_broadcaster;

	initial();
	//connect to serial
	try{
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
	long int fencoderPulse[2] = {0, 0};
	bool initial_flag; 
	initial_flag = true;
	//CurrentEncoder
	vector<unsigned char> lencoder(4);
	vector<unsigned char> rencoder(4);
	long int encoderPulse[2] = {0, 0};

	double Odometry[3] = {0.0};	//x,y,theta (m,m,rad) -pi~pi
	ros::Time current_time;
	while(ros::ok()){
		//ROS_ERROR("FUCK1");

		//Receive data have to delay!! very important!!
		//ros::Duration(0.05).sleep();
    		//ros::Duration(10).sleep();
		array.data.clear();
		array.data.push_back(0x00);
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
		array.data.push_back(0xF4);
		serR.write(array.data);
		//Receive data have to delay!! very important!!
		ros::Duration(0.05).sleep();
		//LeftEncoderInformation
		//RightEncoderInformation
		while(1){
			
			if(serR.available() >= 1){
				//ROS_ERROR("FUCK3");	
				//ROS_INFO_STREAM("Reading from serial port");
				array_.data.clear();
				//read 16 unsigned char
				serR.read(array_.data,1);
				if(array_.data[0]==0x01)
					break;
			}
			else
					ros::Duration(0.1).sleep();
		}
	
		while(1){

			if(serR.available() >= 15){
				array_.data.clear();
				//read 15 unsigned char
				serR.read(array_.data,15);	
				serR.flushInput();
				//if((array_.data[0]==0x66)&&(array_.data[7]==0x02)&&(array_.data[8]==0x66)){

				int CRC=0, CRC2=0;
        unsigned int CRCMotorRcd[6] = { 0x01,array_.data[0],array_.data[1],array_.data[2],array_.data[3], array_.data[4]};
				std::cout<<"CRCMotorRcd=" << CRCMotorRcd[0] << " " << CRCMotorRcd[1] << " " << CRCMotorRcd[2] << " " << CRCMotorRcd[3] << " " << CRCMotorRcd[4] << " " << CRCMotorRcd[5] << "\n";
				unsigned int CRCMotorRcd2[6] = { array_.data[7],array_.data[8],array_.data[9],array_.data[10],array_.data[11], array_.data[12]};
				std::cout<<"CRCMotorRcd2=" << CRCMotorRcd2[0] << " " << CRCMotorRcd2[1] << " " << CRCMotorRcd2[2] << " " << CRCMotorRcd2[3] << " " << CRCMotorRcd2[4] << " " << CRCMotorRcd2[5] << "\n";
				CRC=calc_crc(CRCMotorRcd,6);
				CRC2=calc_crc(CRCMotorRcd2,6);
				std::cout<<"CRC=" << (CRC&0XFF) << " " << (CRC>>8) << " " << (CRC2&0XFF) << " " << (CRC2>>8) <<"\n";
				if((array_.data[5]==(CRC&0XFF))&&(array_.data[6]==(CRC>>8))&&(array_.data[13]==(CRC2&0XFF))&&(array_.data[14]==(CRC2>>8))){
					//std::cout<<"EncoderInfo=" << (int)(unsigned int)array_.data[0] <<" ;" <<(int)(unsigned int)array_.data[1] <<" ;" <<(int)(unsigned int)array_.data[2] <<" ;" << (int)(unsigned int)array_.data[3] <<" ;"<<(int)(unsigned int)array_.data[4] <<" ;" <<(int)(unsigned int)array_.data[5] <<" ;"<<(int)(unsigned int)array_.data[6] <<" ;"<<(int)(unsigned int)array_.data[7] <<" ;"<<(int)(unsigned int)array_.data[8] <<" ;"<<(int)(unsigned int)array_.data[9] <<" ;"<<(int)(unsigned int)array_.data[10] <<" ;"<<(int)(unsigned int)array_.data[11] <<" ;"<<(int)(unsigned int)array_.data[12] <<" ;"<<(int)(unsigned int)array_.data[13] <<" ;"<<(int)(unsigned int)array_.data[14] << "\n";
					for(int i = 0; i < 4; i++){
						rencoder[i] = array_.data[i+1];
						lencoder[i] = array_.data[i+9];
					}			
					break;
				}	
				else 
					break;

			}
		}

		//Compute Odometry
		current_time = ros::Time::now();
		//cout << "Multiple:" << fourthmultiple << " ;" << thirdmultiple << " ;" << secondmultiple << "\n";

		//left
		if ((~lencoder[0]+1) < 0){
			encoderPulse[0] = (lencoder[0] * secondmultiple + lencoder[1] -65536)*10000 +lencoder[2] * secondmultiple + lencoder[3];
		}else{
			encoderPulse[0] = (lencoder[0] * secondmultiple + lencoder[1])*10000 +lencoder[2] * secondmultiple + lencoder[3];
		}
		//right
		if ((~rencoder[0]+1) < 0){
			encoderPulse[1] = (rencoder[0] * secondmultiple + rencoder[1] -65536)*10000 +rencoder[2] * secondmultiple + rencoder[3];
		}else{
			encoderPulse[1] = (rencoder[0] * secondmultiple + rencoder[1])*10000 +rencoder[2] * secondmultiple + rencoder[3];
		}
#if 1
		//initialvalue
		if(initial_flag == true){
			initial_flag = false;
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
		cout << "LencoderPulse : " << encoderPulse[0] << ", RencoderPulse : " << encoderPulse[1] << "\n";
		cout << "LWPulse : " << WPluse[0] << ", RWPulse : " << WPluse[1] << "\n\n";

		//Odometry
		//cout << "Odometry : \n";

		double WV[2] = { 0.0, 0.0 };
		WV[0] = ((double)WPluse[0] / EncoderPlus) * 0.27 * (2.0 * M_PI * WheelRadius) / GearRatio / DriverSampleTime;
		WV[1] = ((double)WPluse[1] / EncoderPlus) * 0.27 * (2.0 * M_PI * WheelRadius) / GearRatio / DriverSampleTime;

		double LinearVelocity = (WV[1] + WV[0]) / 2.0;
		double AngularVelocity = -(WV[1] - WV[0]) / WheelDistance;

		odometry_value = odometry_value + std::abs(LinearVelocity * DriverSampleTime);

		double deltatheta = -(WV[1] - WV[0]) * DriverSampleTime / WheelDistance;
		double deltas = (WV[1] + WV[0]) * DriverSampleTime / 2.0;

		double thetak = Odometry[2];	//theta of last time
		Odometry[2] = (Odometry[2] + deltatheta) ;
                if(Odometry[0] < 20 && Odometry[0] > -20)
		  Odometry[0] = Odometry[0] + deltas * cos((Odometry[2] + thetak) / 2.0) ;
		if(Odometry[1] < 20 && Odometry[1] > -20)
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
		odom_trans.child_frame_id = "base_footprint";

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
                if(Odometry[0] < 20 && Odometry[0] > -20)
		  odom.pose.pose.position.x = Odometry[0];
                if(Odometry[1] < 20 && Odometry[1] > -20)
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
		//ROS_ERROR("FUCK4");
		loop_rate.sleep();

		ros::spinOnce();

	}
	//back  serL.close();
	serR.close();
}


void initial(){
	ros::NodeHandle para_node("~");
	if(!para_node.getParam("baudrate", baudrate))
    		baudrate = 115200;
  	if(!para_node.getParam("Rport", Rport))
    		Rport = "/dev/ttyUSB0";
}

unsigned int calc_crc(unsigned int *buf, int length) {
 unsigned int crc = 0xFFFF;
 int i,j;
 unsigned int LSB;
 for (i = 0; i < length; i++) {
  crc ^= buf[i];
  for (j = 0; j < 8; j++) {
   LSB= crc & 1;
   crc = crc >> 1;
   if (LSB) {
    crc ^= 0xA001;
   }
  }
 }
 return crc;
 //return ((crc & 0xFF00) >> 8)|((crc & 0x0FF) << 8 );
}

void write_serial(const geometry_msgs::Twist::ConstPtr& speed)
{
	unsigned int CRCMotorCmd[15] = { 0x00, 0x65, 0x02, 0x01, 0x0a, 0x00 ,0x00,0x01,0x2c, 0x02, 0x0a, 0x00 ,0x00, 0x00 ,0x00};
	unsigned int MotorCmd[17] = { 0x00, 0x65, 0x02, 0x01, 0x0a, 0x00 ,0x00,0x01,0x2c, 0x02, 0x0a, 0x00 ,0x00,0x00, 0x00 ,0xfe,0xd4};	
	//char r_hex[4];
	//char l_hex[4];
	int CRC=0;
	
	double vr = (speed->linear.x * 2.0 - speed->angular.z * WheelDistance) / 2.0;
	double vl = (speed->linear.x * 2.0 + speed->angular.z * WheelDistance) / 2.0;	
	//double vr = (speed->linear.x * 2.0 + speed->angular.z * WheelDistance) / 2.0;
	//double vl = (speed->linear.x * 2.0 - speed->angular.z * WheelDistance) / 2.0;
	int leftmotorvalue = (int)(round(vl / (2.0 * M_PI *  WheelRadius) * 60.0));
	int rightmotorvalue = (int)(round(vr / (2.0 * M_PI *  WheelRadius) * 60.0));
	//rightmotorvalue=rightmotorvalue*(-1);
	if (leftmotorvalue > 250)
		leftmotorvalue = 250;
	else if (leftmotorvalue < -250)
		leftmotorvalue = -250;

	if (rightmotorvalue > 250)
		rightmotorvalue = 250;
	else if (rightmotorvalue < -250)
		rightmotorvalue = -250;

	int motorLcmd = (int)(round(leftmotorvalue * -15)); //15
	int motorRcmd = (int)(round(rightmotorvalue * 15)); //15
	//std::cout<<"1RMotorRPM = "<< rightmotorvalue <<" ;LMotorRPM = "<< leftmotorvalue << " ;" << " ;motorRcmd = "<< motorRcmd << " ;motorLcmd = "<< motorLcmd << "\n";
   //motorLcmd=motorLcmd*(-1.0);
	//std::cout<<"2RMotorRPM = "<< rightmotorvalue <<" ;LMotorRPM = "<< leftmotorvalue << " ;" << " ;motorRcmd = "<< motorRcmd << " ;motorLcmd = "<< motorLcmd << "\n";
    if(motorRcmd <0)
    	motorRcmd = 0xFFFF + motorRcmd +1;
    if(motorLcmd <0)    
		motorLcmd = 0xFFFF + motorLcmd +1;
	
	//std::cout<<"RMotorRPM = "<< rightmotorvalue <<" ;LMotorRPM = "<< leftmotorvalue << " ;" << " ;motorRcmd = "<< motorRcmd << " ;motorLcmd = "<< motorLcmd << "\n";
	//std::cout<<"RMotorRPM = "<< rightmotorvalue <<" ;LMotorRPM = "<< leftmotorvalue <<"\n";
	MotorCmd[8] = motorRcmd&0XFF;	//low 8
  	MotorCmd[7] = motorRcmd>>8;	//high 8
	MotorCmd[14] = motorLcmd&0XFF;	//low 8
  	MotorCmd[13] = motorLcmd>>8;	//high 8

	CRCMotorCmd[8] = motorRcmd&0XFF;	//low 8
  	CRCMotorCmd[7] = motorRcmd>>8;	//high 8
	CRCMotorCmd[14] = motorLcmd&0XFF;	//low 8
  	CRCMotorCmd[13] = motorLcmd>>8;	//high 8

	CRC=calc_crc(CRCMotorCmd,15);
	//ROS_ERROR("%d",CRC);
	MotorCmd[15] = CRC&0XFF;	//low 8
  	MotorCmd[16] = CRC>>8;	//high 8


	//std::cout<<MotorCmd[0]<<";"<<MotorCmd[1]<<";"<<MotorCmd[2]<<";"<<MotorCmd[3]<<";"<<MotorCmd[4]<<";"<<MotorCmd[5]<<";"<<MotorCmd[6]<<";"<<MotorCmd[7]<<";"<<MotorCmd[8]<<";"<<MotorCmd[9]<<";"<<MotorCmd[10]<<";"<<MotorCmd[11]<<";" <<MotorCmd[12]<<";"<<MotorCmd[13]<<";"<<MotorCmd[14]<<";"<<MotorCmd[15]<<";"<<MotorCmd[16]<<";"<< "\n";



	std_msgs::UInt8MultiArray array;

	array.data.clear();

		array.data.push_back(MotorCmd[0]);
		array.data.push_back(MotorCmd[1]);
		array.data.push_back(MotorCmd[2]);
		array.data.push_back(MotorCmd[3]);
		array.data.push_back(MotorCmd[4]);
		array.data.push_back(MotorCmd[5]);
		array.data.push_back(MotorCmd[6]);
		array.data.push_back(MotorCmd[7]);
		array.data.push_back(MotorCmd[8]);
		array.data.push_back(MotorCmd[9]);
		array.data.push_back(MotorCmd[10]);
		array.data.push_back(MotorCmd[11]);
		array.data.push_back(MotorCmd[12]);
		array.data.push_back(MotorCmd[13]);
		array.data.push_back(MotorCmd[14]);
		array.data.push_back(MotorCmd[15]);
		array.data.push_back(MotorCmd[16]);


	serR.write(array.data);
	ros::Duration(0.05).sleep();


}

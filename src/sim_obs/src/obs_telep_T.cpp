

#include <iostream>
#include <map>
#include <math.h>

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <gazebo_msgs/ModelStates.h>

#include <termios.h> 
#include <unistd.h> 
#include <fcntl.h>
#include <iostream>
#include <signal.h>

using namespace std;

/********************/
/* CLASS DEFINITION */
/********************/
class Obs_telep
{
    public:
        Obs_telep();
        ~Obs_telep();
        void start(void);
        
        
    private:
        ros::Publisher obs_telep_pub;
        map<char,int> mapkey;
        double key_matrix[4][3];
        char ch;
        map<char,int>::iterator it;
        bool debounced = 0; // 0 for not press, 1 for has press
        const int debounced_time = 10;
        int kbhit(void);

}; // end of class


Obs_telep::Obs_telep()
{
    //Private parameters handler
    ros::NodeHandle node_handle;
    //Publishers and Subscribers
    obs_telep_pub = node_handle.advertise<geometry_msgs::Twist>("/obsvel_plannar_T", 1);
    mapkey['w'] = 0;
    mapkey['s'] = 1;
    mapkey['a'] = 2;
    mapkey['d'] = 3;

    // w for forward x
    key_matrix[0][0] = 0;
    key_matrix[0][1] = -0.3;
    key_matrix[0][2] = 0;

    //  s for backward x
    key_matrix[1][0] = 0;
    key_matrix[1][1] = 0.3;
    key_matrix[1][2] = 0;

    // a for forward x
    key_matrix[2][0] = 0.3;
    key_matrix[2][1] = 0.0;
    key_matrix[2][2] = 0;

    //  d for backward x
    key_matrix[3][0] = -0.3;
    key_matrix[3][1] = -0.0;
    key_matrix[3][2] = 0;
    
}

Obs_telep::~Obs_telep()
{
};

int Obs_telep:: kbhit(void) 
{ 
  /*
    wait until keyboard press function
    reference : https://blog.csdn.net/lanmanck/article/details/5823562
  */
  struct termios oldt, newt; 
  int ch; 
  int oldf; 
  tcgetattr(STDIN_FILENO, &oldt); 
  newt = oldt; 
  newt.c_lflag &= ~(ICANON | ECHO); 
  tcsetattr(STDIN_FILENO, TCSANOW, &newt); 
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0); 
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK); 
  ch = getchar(); 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt); 
  fcntl(STDIN_FILENO, F_SETFL, oldf); 
  if(ch != EOF) { 
      ungetc(ch, stdin); 
      return 1; 
  } 
  return 0; 
} 



void Obs_telep::start()
{
    ros::Rate rate(30);
    int counter = 0;
    geometry_msgs::Twist _twist_msg = geometry_msgs::Twist();
    while (ros::ok()) {
        

        while(!kbhit()){
            if(counter > debounced_time){
            debounced = 0;
            counter = 0;
            }else{
                counter ++;
            }
            // cout << debounced << endl;
        }
        if ((ch = getchar()) && debounced == 0){
            if(ch == 'p' || ch == 'P'){
                _twist_msg.linear.x  = 0; 
                _twist_msg.linear.y  = 0; 
                _twist_msg.linear.z  = 0; 
                obs_telep_pub.publish(_twist_msg);
            }
            else
            {
                // cout << ch << endl;
                it = mapkey.find(ch);
                // cout << it->second <<endl;
                // cout << "x : "  << key_matrix[it->second][0] << ", y : " << key_matrix[it->second][1] << endl; 
                if(it!=mapkey.end())
                {
                    _twist_msg.linear.x  += key_matrix[it->second][0]; 
                    _twist_msg.linear.y  += key_matrix[it->second][1]; 
                    _twist_msg.linear.z  += key_matrix[it->second][2]; 
                    obs_telep_pub.publish(_twist_msg);
                }else{
                    cout << "only w, s, a, d, p" << endl;
                }
            }
            
            
            debounced = 1;     
        }
        
        rate.sleep();
        ros::spinOnce();
    }
    
}

/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "obsvel_plannar_T");
    Obs_telep obs;
    
    obs.start();
        
    
    
    ros::waitForShutdown();
    return 0;
}

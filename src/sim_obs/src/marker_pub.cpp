

#include <iostream>
#include <map>
#include <math.h>

#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>
 
// #include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
//#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <gazebo_msgs/ModelStates.h>

#include <Eigen/Core>
#include <Eigen/QR>

// inlcude iostream and string libraries
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>  
#include <fstream>
#include <vector>


#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

using namespace std;
using namespace Eigen;
using namespace chrono;



typedef struct Pose
{
    double x,y,z;
    double vx,vy,vz,vw;
};

typedef struct modelState
{
    Pose pose;
    string itemName;
};

/********************/
/* CLASS DEFINITION */
/********************/
class MPCNode
{
    public:
        MPCNode();
        ~MPCNode();
        void start(void);
        
    private:
        void callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
        ros::Publisher vis_pub,vis_pub2;
        ros::Subscriber gazebo_sub;
        std::vector<modelState> modelStateVector;
        double x=0,y=0,z=0;
        double x2=0,y2=0,z2=0;
    
        

}; // end of class


MPCNode::MPCNode()
{
    //Private parameters handler
    ros::NodeHandle node_handle;
    //Publishers and Subscribers
    vis_pub2 = node_handle.advertise<visualization_msgs::MarkerArray>( "detected_obstacle2", 0 );
    vis_pub = node_handle.advertise<visualization_msgs::Marker>( "detected_obstacle", 0 );
    gazebo_sub = node_handle.subscribe("/gazebo/model_states",1,&MPCNode::callback,this);
}

MPCNode::~MPCNode()
{
};

void MPCNode::callback(const gazebo_msgs::ModelStates::ConstPtr& msg){
    int find_index1 = 0;
    int find_index2 = 0;
    modelStateVector.clear();
    // for (int i = 0; i < msg->name.size(); i++)
    // {
    //     /* code */
    //     if(!msg->name[i].compare("move_unit_cylinder")){
    //         find_index1 = i;
    //         // cout << "--------------- "<< msg->name[i];
    //     }
    //     if(!msg->name[i].compare("unit_cylinder2")){
    //         find_index2 = i;
    //         // cout << "--------------- "<< msg->name[i];
    //     }
    // }
    // // cout << "--------------- "<< find_index << " ---------\n";
    
    // x  = msg->pose[find_index1].position.x;
    // y  = msg->pose[find_index1].position.y;
    // z  = msg->pose[find_index1].position.z;
    
    // x2 = msg->pose[find_index2].position.x;
    // y2 = msg->pose[find_index2].position.y;
    // z2 = msg->pose[find_index2].position.z;
    for (int i = 0; i < msg->name.size(); i++){
        
        if(!msg->name[i].compare("ground_plane") || !msg->name[i].compare("isci_robot")){
            continue;
        }
        else{            
            modelState model;
            model.itemName = msg->name[i];
            model.pose.x=msg->pose[i].position.x;
            model.pose.y=msg->pose[i].position.y;
            model.pose.z=msg->pose[i].position.z;
            model.pose.vx= msg->pose[i].orientation.x;
            model.pose.vy= msg->pose[i].orientation.y;
            model.pose.vz= msg->pose[i].orientation.z;
            model.pose.vw= msg->pose[i].orientation.w;
            modelStateVector.push_back(model);
        }
    }
}

void MPCNode::start()
{
    // ros::Rate rate(20);
    // int i = 1;
    // while (ros::ok()) {
    //     try
    //     {
    //         boost::interprocess::shared_memory_object shdmem(boost::interprocess::open_only, "OBSshm", boost::interprocess::read_only);
    //         boost::interprocess::mapped_region region2(shdmem, boost::interprocess::read_only);
    //         // std::cout << std::hex << region2.get_address() << std::endl;
    //         // std::cout << std::dec << region2.get_size() << std::endl;
    //         nav_msgs::Odometry* outObsData = static_cast<nav_msgs::Odometry*>(region2.get_address());
    //         // std::cout << outObsData->pose.pose.position.x << std::endl<< std::endl;

    //         // visualization_msgs::MarkerArray marker_all;
    //         visualization_msgs::Marker marker;
    //         marker.header.frame_id = "odom";
    //         marker.header.stamp = ros::Time::now();
    //         marker.ns = "my_namespace";
    //         marker.scale.x = 0.2;
    //         marker.scale.y = 0.2;
    //         marker.scale.z = 0.2;
    //         marker.color.a = 1.0; // Don't forget to set the alpha!
    //         marker.color.r = 0.0;
    //         marker.color.g = 1.0;
    //         marker.color.b = 0.0;

            
    //         marker.id = 1;            
    //         marker.action = visualization_msgs::Marker::ADD;
            
    //         marker.type = visualization_msgs::Marker::SPHERE;

    //         marker.lifetime = ros::Duration();
            
    //         marker.pose.position.x = outObsData->pose.pose.position.x;
    //         marker.pose.position.y = outObsData->pose.pose.position.y;
    //         marker.pose.position.z = outObsData->pose.pose.position.z;
    //         marker.pose.orientation.x = 0;
    //         marker.pose.orientation.y = 0;
    //         marker.pose.orientation.z = 0;
    //         marker.pose.orientation.w = 1;

    //         // marker_all.markers.clear();
    //         // marker_all.markers.push_back(marker);
            
    //         vis_pub.publish( marker );
    //         i++;

    //     }
    //     catch(const std::exception& e)
    //     {
    //         std::cerr << e.what() << '\n';
    //     }
    //     // if(i%20 == 0){
    //     //     visualization_msgs::MarkerArray marker_all;
    //     //     visualization_msgs::Marker marker;
    //     //     marker.header.frame_id = "world";
    //     //     marker.header.stamp = ros::Time();
    //     //     marker.ns = "my_namespace";

            
    //     //     marker.id = 1;            
    //     //     marker.action = visualization_msgs::Marker::DELETE;
            
    //     //     marker.type = visualization_msgs::Marker::SPHERE;
            
    //     //     marker_all.markers.push_back(marker);
            
    //     //     vis_pub.publish( marker_all );
    //     //     i = 1;
    //     // }
        
    //     rate.sleep();
    // }
    int i =0;
    ros::Rate rate(60);
    while (ros::ok()) {


        try
        {
            boost::interprocess::shared_memory_object shdmem(boost::interprocess::open_only, "OBSshm", boost::interprocess::read_only);
            boost::interprocess::mapped_region region2(shdmem, boost::interprocess::read_only);
            // std::cout << std::hex << region2.get_address() << std::endl;
            // std::cout << std::dec << region2.get_size() << std::endl;
            nav_msgs::Odometry* outObsData = static_cast<nav_msgs::Odometry*>(region2.get_address());
            // std::cout << outObsData->pose.pose.position.x << std::endl<< std::endl;

            // visualization_msgs::MarkerArray marker_all;
            visualization_msgs::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = ros::Time::now();
            marker.ns = "my_namespace";
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            
            marker.id = 1;            
            marker.action = visualization_msgs::Marker::ADD;
            
            marker.type = visualization_msgs::Marker::SPHERE;

            marker.lifetime = ros::Duration();
            
            marker.pose.position.x = outObsData->pose.pose.position.x;
            marker.pose.position.y = outObsData->pose.pose.position.y;
            marker.pose.position.z = outObsData->pose.pose.position.z;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
            marker.pose.orientation.w = 1;

            // marker_all.markers.clear();
            // marker_all.markers.push_back(marker);
            
            vis_pub.publish( marker );
            // i++;

        }
        catch(const std::exception& e)
        {
            // std::cerr << "shared memory " << e.what() << '\n';
        }
        // if(i%20 == 0){
        //     visualization_msgs::MarkerArray marker_all;
        //     visualization_msgs::Marker marker;
        //     marker.header.frame_id = "world";
        //     marker.header.stamp = ros::Time();
        //     marker.ns = "my_namespace";

            
        //     marker.id = 1;            
        //     marker.action = visualization_msgs::Marker::DELETE;
            
        //     marker.type = visualization_msgs::Marker::SPHERE;
            
        //     marker_all.markers.push_back(marker);
            
        //     vis_pub.publish( marker_all );
        //     i = 1;
        // }


        visualization_msgs::MarkerArray marker_all;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        // marker.id = 0;
        // marker.type = visualization_msgs::Marker::CYLINDER;
        // marker.action = visualization_msgs::Marker::ADD;

        // marker.pose.position.x = x;
        // marker.pose.position.y =  y;
        // marker.pose.position.z =  z;
        // marker.pose.orientation.x = 0.0;
        // marker.pose.orientation.y = 0.0;
        // marker.pose.orientation.z = 0.0;
        // marker.pose.orientation.w = 1.0;
        // marker.scale.x = 0.5;
        // marker.scale.y = 0.5;
        // marker.scale.z = 1.8;
        // marker.color.a = 0.5; // Don't forget to set the alpha!
        // marker.color.r = 0.4;
        // marker.color.g = 0.4;
        // marker.color.b = 0.4;
        
        // //only if using a MESH_RESOURCE marker type:
        // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        // marker_all.markers.push_back(marker);

        // marker.id = 1;
        // marker.pose.position.x = x2;
        // marker.pose.position.y = y2;
        // marker.pose.position.z = z2;
        // marker_all.markers.push_back(marker);
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 1.8;
        marker.color.a = 0.8; // Don't forget to set the alpha!
        marker.color.r = 0.4;
        marker.color.g = 0.4;
        marker.color.b = 0.4;

        for (int ik = 0; ik < modelStateVector.size(); ik++)
        {
            marker.id = ik;            
            marker.action = visualization_msgs::Marker::ADD;
            if(!modelStateVector[ik].itemName.compare("T") || !modelStateVector[ik].itemName.compare("T2")){
                
                marker.type = visualization_msgs::Marker::MESH_RESOURCE;
                marker.mesh_resource = "package://sim_obs/mesh/T.stl";
                marker.scale.x = 0.001;
                marker.scale.y = 0.001;
                marker.scale.z = 0.001;
            }else if(!modelStateVector[ik].itemName.compare("Desk")){
                marker.type = visualization_msgs::Marker::MESH_RESOURCE;
                marker.mesh_resource = "package://sim_obs/mesh/Desk.stl";
                marker.scale.x = 0.001;
                marker.scale.y = 0.001;
                marker.scale.z = 0.001;
            }else{
                marker.type = visualization_msgs::Marker::CYLINDER;
            }
            marker.pose.position.x = modelStateVector[ik].pose.x;
            marker.pose.position.y =  modelStateVector[ik].pose.y;
            marker.pose.position.z =  modelStateVector[ik].pose.z;
            marker.pose.orientation.x = modelStateVector[ik].pose.vx;
            marker.pose.orientation.y = modelStateVector[ik].pose.vy;
            marker.pose.orientation.z = modelStateVector[ik].pose.vz;
            marker.pose.orientation.w = modelStateVector[ik].pose.vw;
            marker_all.markers.push_back(marker);
        }

        // if(i%20 == 0){
        //     visualization_msgs::MarkerArray marker_all;
        //     visualization_msgs::Marker marker;
        //     marker.header.frame_id = "odom";
        //     marker.header.stamp = ros::Time();
        //     marker.ns = "my_namespace";

            
        //     marker.id = 1;            
        //     marker.action = visualization_msgs::Marker::DELETE;
            
        //     marker.type = visualization_msgs::Marker::SPHERE;
            
        //     marker_all.markers.push_back(marker);
            
        //     vis_pub2.publish( marker_all );
        //     i = 1;
        // }
        

        vis_pub2.publish( marker_all );
        rate.sleep();
    }
    
}

/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "marker");
    MPCNode mpc_node;
    ros::AsyncSpinner spinner(2); // Use multi threads
    spinner.start();
    
    mpc_node.start();
        
    
    
    ros::waitForShutdown();
    return 0;
}

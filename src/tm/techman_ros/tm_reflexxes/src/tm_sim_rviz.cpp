/*********************************************************************
 * tm_bringup_reflexxes.cpp
 *
 * Copyright (c) 2017, ISCI / National Chiao Tung University (NCTU)
 *
 * Author: Howard Chen (s880367@gmail.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************************************************************
 *
 * Author: Howard Chen
 */

/* Based on original source from Yun-Hsuan Tsai */

/*********************************************************************
 * tm_ros_wrapper.cpp
 *
 * Copyright 2016 Copyright 2016 Techman Robot Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************************************************************/

#include <stdio.h>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "tm_reflexxes/tm_reflexxes.h"

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>



#define DEG2RAD 0.01745329252
#define RAD2DEG 57.29577951

#define CYCLE_TIME_IN_SECONDS                   0.025
#define NUMBER_OF_DOFS                          6

#define MAX_ACC 0.0375*40 // 0.0375 : acc in 25ms

std::vector<double> jointposition(6), jointvelocity(6), jointeffort(6);

void publishMsgRT();
bool ReflexxesPositionRun_sim(  RMLPositionInputParameters &InputState, 
                                std::vector<double> TargetPosition,
                                std::vector<double> TargetVelocity, 
                                double SynTime);

int main(int argc, char **argv)
{
    ros::init(argc, argv,"tm_otg");

    bool run_succeed = true;
    double SynchronousTime = 3.0;
    std::vector<double> TargetPosition, TargetVelocity;
    boost::thread state_publisher;

    RMLPositionInputParameters  *IP_position = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    RMLVelocityInputParameters  *IP_velocity = new RMLVelocityInputParameters(NUMBER_OF_DOFS);

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP_position->CurrentPositionVector->VecData[i] = 0.0;
        IP_position->CurrentVelocityVector->VecData[i] = 0.0;
        IP_position->CurrentAccelerationVector->VecData[i] = 0.0;

        IP_velocity->CurrentPositionVector->VecData[i] = 0.0;
        IP_velocity->CurrentVelocityVector->VecData[i] = 0.0;
        IP_velocity->CurrentAccelerationVector->VecData[i] = 0.0;
    }

    state_publisher = boost::thread(publishMsgRT);

    while(ros::ok())
    {
        if (run_succeed)
        {
            TargetPosition = {0,0,1.57,-1.57,1.57,0};
            TargetVelocity = {0, 0, 0, 0, 0, 0};
            run_succeed = ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime);
        }
        else
            break;

        if (run_succeed)
        {
            TargetPosition = {0,0,0,0,0,0};
            TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
            run_succeed = ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime);
        }
        else
            break;
    }

    state_publisher.join();

    delete IP_position;
    delete IP_velocity;

    return 0;
}



void publishMsgRT()
{
    ros::NodeHandle nh_;
    std::vector<double> joint_offsets;
    std::string base_frame;
    std::string tool_frame;
    static tf::TransformBroadcaster tf_bc;
    sensor_msgs::JointState joint_msg;
    std::vector<std::string> joint_names;

    joint_offsets.assign(6, 0.0);
    std::string joint_prefix = "";

    joint_names.push_back(joint_prefix + "shoulder_1_joint");
    joint_names.push_back(joint_prefix + "shoulder_2_joint");
    joint_names.push_back(joint_prefix + "elbow_1_joint");
    joint_names.push_back(joint_prefix + "wrist_1_joint");
    joint_names.push_back(joint_prefix + "wrist_2_joint");
    joint_names.push_back(joint_prefix + "wrist_3_joint");
    base_frame = joint_prefix + "base_link";
    tool_frame = joint_prefix + "tool0";

    ros::Publisher joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 40);
    joint_msg.name = joint_names;
    ros::Rate loop_rate(40);

    while (ros::ok())
    {

        double robot_state_time;
        std::vector<double> robot_state_vec;

        //Publish JointState
        joint_msg.header.stamp = ros::Time::now();
        
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            joint_msg.position.push_back(jointposition[i]);
            joint_msg.velocity.push_back(jointvelocity[i]);
            joint_msg.effort.push_back(jointeffort[i]);   
        }
        for (unsigned int i = 0; i < joint_msg.position.size(); i++)
        {
            joint_msg.position[i] += joint_offsets[i];
        }

        joint_pub.publish(joint_msg);

        loop_rate.sleep();
        joint_msg.position.clear();
        joint_msg.velocity.clear();
        joint_msg.effort.clear();
/*
        //Broadcast transform (tool0)
        robot_state_time = rros::Time::now();
        quat.setRPY(robot_state_vec[3], robot_state_vec[4], robot_state_vec[5]);
        tool_pose_msg.header.frame_id = base_frame;
        tool_pose_msg.header.stamp = joint_msg.header.stamp;
        tool_pose_msg.pose.position.x = robot_state_vec[0];
        tool_pose_msg.pose.position.y = robot_state_vec[1];
        tool_pose_msg.pose.position.z = robot_state_vec[2];
        tool_pose_msg.pose.orientation.x = quat.x();
        tool_pose_msg.pose.orientation.y = quat.y();
        tool_pose_msg.pose.orientation.z = quat.z();
        tool_pose_msg.pose.orientation.w = quat.w();
        tool_pos_pub.publish(tool_pose_msg);
        transform.setOrigin(tf::Vector3(
                                        robot_state_vec[0],
                                        robot_state_vec[1],
                                        robot_state_vec[2]));
        transform.setRotation(quat);
        tf_bc.sendTransform(tf::StampedTransform(transform, joint_msg.header.stamp, base_frame, tool_frame));
*/
    }
}
bool ReflexxesPositionRun_sim(  RMLPositionInputParameters &InputState, 
                                std::vector<double> TargetPosition,
                                std::vector<double> TargetVelocity, 
                                double SynTime)
{
    double time_s;
    std::vector<double> FinalPosition;
    bool pass = true;

    ReflexxesAPI *RML = NULL    ;
    RMLPositionInputParameters  *IP = NULL;
    RMLPositionOutputParameters *OP = NULL;
    RMLPositionFlags Flags;
    int ResultValue = 0;

    RML = new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
    IP = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    OP = new RMLPositionOutputParameters(NUMBER_OF_DOFS);

    *IP = InputState;

    //  ********************************************************************/
    //  Assigning all RMLPositionInputParameters : 
    //  Current POS, VEL, ACC : set before call ReflexxesPositionRun
    //  Target POS, VEL       : set before call ReflexxesPositionRun
    //  Max VEL, ACC          : set after call ReflexxesPositionRun
    //  SelectionVector       : set after call ReflexxesPositionRun
    //  ********************************************************************
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP->MaxJerkVector->VecData[i] = 100; //RMLTypeII not using, needed for validity
        IP->MaxVelocityVector->VecData[i] = MAX_VELOCITY; //0.3247
        IP->MaxAccelerationVector->VecData[i] = MAX_ACC;
        IP->TargetPositionVector->VecData[i] = TargetPosition[i]; 
        IP->TargetVelocityVector->VecData[i] = TargetVelocity[i];
        IP->SelectionVector->VecData[i] = true;
    }
    IP->MinimumSynchronizationTime = SynTime;


    if (IP->CheckForValidity())
        printf("Input values are valid!\n");
    else
    {
        printf("Input values are INVALID!\n");
        pass = false;
    }


    struct timeval tm1,tm2, tm3, tm4;
    double cycle_iteration = 1.0;


    gettimeofday(&tm3, NULL);

    while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED && ros::ok())
    {
        //********************************************************
        // The area execution in 25ms real time sharp

        gettimeofday(&tm1, NULL); 

        ResultValue =  RML->RMLPosition(*IP, OP, Flags );

        if (ResultValue < 0)
        {
            printf("An error occurred (%d).\n", ResultValue );
            break;
        }

        //***************************************************************
        // Print out commands

        time_s = cycle_iteration*0.025;
        cycle_iteration++;

        printf("[ %lf ] pos:  ",time_s);
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            printf("%10.4lf ", OP->NewPositionVector->VecData[i]);
            jointposition[i] = OP->NewPositionVector->VecData[i];
        }

        printf(" | spd: ");

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            printf("%10.4lf ", OP->NewVelocityVector->VecData[i]);
            jointvelocity[i] = OP->NewVelocityVector->VecData[i];
            jointeffort[i] = OP->NewAccelerationVector->VecData[i];
        }
        printf("\n");


        //***************************************************************
/*
        if (tm_reflexxes::kbhit())
        {
            char c = getchar();
            if (c == 'q' || c == 'Q')
            {
                print_info("Smooth Stop Activate...");
                RMLVelocityInputParameters *IP_vel = new RMLVelocityInputParameters(NUMBER_OF_DOFS);

                *IP_vel->CurrentPositionVector     = *IP->CurrentPositionVector;
                *IP_vel->CurrentVelocityVector     = *IP->CurrentVelocityVector;
                *IP_vel->CurrentAccelerationVector = *IP->CurrentAccelerationVector;
                tm_reflexxes::ReflexxesSmoothStop_sim(*IP_vel, 0.5);
                delete IP_vel;
                pass = false;
                break;
            }
        }
*/
        *IP->CurrentPositionVector =  *OP->NewPositionVector;
        *IP->CurrentVelocityVector =  *OP->NewVelocityVector;

        gettimeofday(&tm2, NULL);
        long long time_compensation = 1000000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec);            
        usleep(24940 - time_compensation);  

        //********************************************************
        // The area execution in 25ms real time sharp
    }

    gettimeofday(&tm4, NULL);
    long long tt = 1000000 * (tm4.tv_sec - tm3.tv_sec) + (tm4.tv_usec - tm3.tv_usec);

    if(pass)
    {
        printf("=============== Final state =========================\n");

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            printf("%10.4lf ", IP->CurrentPositionVector->VecData[i]);

        printf("\n");
        print_info("Finished in %llu us", tt);
    }
    //tm_reflexxes::resetTermios();
    InputState = *IP;

    delete  RML;
    delete  IP;
    delete  OP;

    return pass;
}
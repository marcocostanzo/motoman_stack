#include <stdlib.h> 
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <sstream>
#include <fstream>
#include <vector>

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"

#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "sensor_msgs/JointState.h"

#include <TooN/TooN.h>

#define NUM_JOINTS 7

using namespace std;
using namespace TooN;

vector<string> default_joint_names;
Vector<> q = Zeros(NUM_JOINTS); //Measured joint positions (updated in the readJointPos callback)
Vector<> qp = Zeros(NUM_JOINTS);

Vector<> qd = Zeros(NUM_JOINTS), qpd = Zeros(NUM_JOINTS);

/* Callback for joint state.*/
void readJointPos(const sensor_msgs::JointState jointStateMsg)
{
//  if(jointStateMsg.name.size() != NUM_JOINTS) {
//		cout << "Start and End position joint_names mismatch." << endl;
//		exit(0);
//	}
//	else {

    if (jointStateMsg.name.size() == NUM_JOINTS){
		for(int i = 0; i < NUM_JOINTS; i++) {
//			if( strcmp(jointStateMsg.name[i].c_str(), default_joint_names[i].c_str()) != 0 ){
//				cout << "Start and End position joint_names mismatch" << endl;
//				exit(0);
//			} else {
				q[i] = jointStateMsg.position[i];
//			}
		}
         }
//	}
}

/* Callback for joint command.*/
void readCommand(const std_msgs::Float64MultiArray jointCommandMsg)
{

	for(int i = 0; i < NUM_JOINTS; i++) {
		
		qd[i] = jointCommandMsg.data[i];
		qpd[i] = jointCommandMsg.data[i+NUM_JOINTS];
	}
}

int main(int argc, char **argv) {

	/* ROS node initialization.*/
	int _argc = 0;
	char** _argv = NULL;
	ros::init(_argc,_argv, "motoman_control_node");

	if(!ros::master::check())
		return(0);

	ros::NodeHandle n;
 
  /* Publisher.*/
	ros::Publisher joint_cmd_pub = n.advertise<trajectory_msgs::JointTrajectory>("joint_command", 1);
	ros::Publisher joint_states_pub = n.advertise<std_msgs::Float64MultiArray>("simple_joint_states", 1);
	std_msgs::Float64MultiArray joint_states_msg;
	joint_states_msg.data.resize(7);

	/* Subscriber.*/
	ros::Subscriber joint_states_sub = n.subscribe("joint_states", 1, readJointPos);
	ros::Subscriber ll_control_sub = n.subscribe("joint_ll_control", 1, readCommand);

  float rate = 50.0;		/* 50 Hz */
	ros::Rate loop_rate(rate); 

	/* NODE inizialization.*/
	trajectory_msgs::JointTrajectoryPoint jointTrajPoint;
	trajectory_msgs::JointTrajectory jointTraj;
	jointTrajPoint.positions.resize(7);
	jointTrajPoint.velocities.resize(7);
	jointTraj.points.resize(1);
		
  cout << "Starting motion command." << endl;
	
	if(NUM_JOINTS == 6){
		default_joint_names.push_back("joint_s");
		default_joint_names.push_back("joint_l");
		default_joint_names.push_back("joint_u");
		default_joint_names.push_back("joint_r");
		default_joint_names.push_back("joint_b");
		default_joint_names.push_back("joint_t");
	} else if(NUM_JOINTS == 7){
		default_joint_names.push_back("joint_s");
		default_joint_names.push_back("joint_l");
		default_joint_names.push_back("joint_e");
		default_joint_names.push_back("joint_u");
		default_joint_names.push_back("joint_r");
		default_joint_names.push_back("joint_b");
		default_joint_names.push_back("joint_t");
	}

	sleep(1);
	ros::spinOnce();
	
	/* Trajectory initialization */
	for (int i = 0; i < NUM_JOINTS; i++) 
	{	
		qd[i]=q[i];
		qpd[i]=0.0;
	}
	jointTraj.joint_names = default_joint_names;

  /* Control loop */
	double secs = 0.0;
	double secs_in = ros::Time::now().toSec();
	while (ros::ok()) {

		secs = ros::Time::now().toSec() - secs_in;

		for (int i = 0; i < NUM_JOINTS; i++){

			jointTrajPoint.positions[i] = qd[i];
			jointTrajPoint.velocities[i] = qpd[i];
			
			joint_states_msg.data[i] = q[i];
		}
		jointTrajPoint.time_from_start = ros::Duration(secs);
	
  	/* Publish message */
  	jointTraj.points.at(0) = jointTrajPoint;
  	joint_cmd_pub.publish(jointTraj);
		joint_states_pub.publish(joint_states_msg);

		cout << "." << flush;

  	ros::spinOnce();	
     
		loop_rate.sleep();
	}

	return(0);
}

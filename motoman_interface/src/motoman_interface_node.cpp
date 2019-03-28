#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include "motoman_interface/JointPositionVelocity.h"

#define NUM_JOINTS 7

using namespace std;

/* Global ROS vars*/
ros::Publisher out_joint_command_pub;
trajectory_msgs::JointTrajectory out_jointTraj_msg;
/* END Global ROS vars*/

/* ROS CALLBK */

void readJointCommand(const motoman_interface::JointPositionVelocity::ConstPtr& jointCommandMsg){

    out_jointTraj_msg.points[0].positions[0] = jointCommandMsg->position.s;
    out_jointTraj_msg.points[0].positions[1] = jointCommandMsg->position.l;
    out_jointTraj_msg.points[0].positions[2] = jointCommandMsg->position.e;
    out_jointTraj_msg.points[0].positions[3] = jointCommandMsg->position.u;
    out_jointTraj_msg.points[0].positions[4] = jointCommandMsg->position.r;
    out_jointTraj_msg.points[0].positions[5] = jointCommandMsg->position.b;
    out_jointTraj_msg.points[0].positions[6] = jointCommandMsg->position.t;

    out_jointTraj_msg.points[0].velocities[0] = jointCommandMsg->velocity.s;
    out_jointTraj_msg.points[0].velocities[1] = jointCommandMsg->velocity.l;
    out_jointTraj_msg.points[0].velocities[2] = jointCommandMsg->velocity.e;
    out_jointTraj_msg.points[0].velocities[3] = jointCommandMsg->velocity.u;
    out_jointTraj_msg.points[0].velocities[4] = jointCommandMsg->velocity.r;
    out_jointTraj_msg.points[0].velocities[5] = jointCommandMsg->velocity.b;
    out_jointTraj_msg.points[0].velocities[6] = jointCommandMsg->velocity.t;

    out_joint_command_pub.publish(out_jointTraj_msg);

}

/* END ROS CALLBK */

int main(int argc, char *argv[])
{
    
    ros::init(argc,argv, "motoman_interface");

    ros::NodeHandle nh_private = ros::NodeHandle("~");
    ros::NodeHandle nh_public = ros::NodeHandle();

    /**** CHECK PARAMS ****/
    string in_joint_command_topic_str = string("");
    nh_private.param("in_joint_command_topic" , in_joint_command_topic_str, string("simple_joint_command") );
    string out_joint_command_topic_str = string("");
    nh_private.param("out_joint_command_topic" , out_joint_command_topic_str, string("joint_command") );

    /* Subscriber.*/
	ros::Subscriber in_joint_command_sub = nh_public.subscribe(in_joint_command_topic_str, 1, readJointCommand);

    /* Publisher.*/
	out_joint_command_pub = nh_public.advertise<trajectory_msgs::JointTrajectory>(out_joint_command_topic_str, 1);

    /*ROSMSGs*/
    out_jointTraj_msg.points.resize(1);
    out_jointTraj_msg.points[0].positions.resize(NUM_JOINTS);
    out_jointTraj_msg.points[0].velocities.resize(NUM_JOINTS);
    out_jointTraj_msg.joint_names.push_back("joint_s");
	out_jointTraj_msg.joint_names.push_back("joint_l");
	out_jointTraj_msg.joint_names.push_back("joint_e");
	out_jointTraj_msg.joint_names.push_back("joint_u");
	out_jointTraj_msg.joint_names.push_back("joint_r");
	out_jointTraj_msg.joint_names.push_back("joint_b");
	out_jointTraj_msg.joint_names.push_back("joint_t");
    out_jointTraj_msg.points[0].time_from_start = ros::Duration(0.0);

    /*SPIN*/
    ros::spin();
    
    return 0;
}
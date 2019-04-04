#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include "motoman_interface/JointPositionVelocity.h"
#include "motoman_interface/GetJoints.h"

#ifndef SUN_COLORS
#define SUN_COLORS

/* ======= COLORS ========= */
#define CRESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLD    "\033[1m"       /* Bold */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */
/*===============================*/

#endif

#define HEADER_PRINT BOLDYELLOW "[" << ros::this_node::getName() << "] " CRESET 

#define NUM_JOINTS 7
#define COMMAND_RATE 50

using namespace std;

/* Global ROS vars*/
ros::Publisher out_joint_command_pub;
trajectory_msgs::JointTrajectory out_jointTraj_msg;
string joint_state_topic_str;
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

}

sensor_msgs::JointState joint_state_msg;
bool b_joint_state_arrived = false;
void readJointState( const sensor_msgs::JointState& joi_state_msg ){
    joint_state_msg = joi_state_msg;
    b_joint_state_arrived = true;
}

bool getJoints(motoman_interface::GetJoints::Request  &req, 
   		 		motoman_interface::GetJoints::Response &res){

    cout << HEADER_PRINT "Service getJoints()" << endl;

    res.position.s = joint_state_msg.position[0];
    res.position.l = joint_state_msg.position[1];
    res.position.e = joint_state_msg.position[2];
    res.position.u = joint_state_msg.position[3];
    res.position.r = joint_state_msg.position[4];
    res.position.b = joint_state_msg.position[5];
    res.position.t = joint_state_msg.position[6];
    
    res.success = true;
    return true;
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
    nh_private.param("joint_state_topic" , joint_state_topic_str, string("joint_states") );
    string service_get_joints_str;
    nh_private.param("get_joints_service" , service_get_joints_str, string("getJoints") );

    /* Subscriber.*/
	ros::Subscriber in_joint_command_sub = nh_public.subscribe(in_joint_command_topic_str, 1, readJointCommand);
    ros::Subscriber joint_position_sub = nh_public.subscribe(joint_state_topic_str, 1, readJointState);

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

    //wait joint state
    cout << HEADER_PRINT "Wait initial joint position..." << endl;
    while( ros::ok() && !b_joint_state_arrived ){
        ros::spinOnce();
    }
    cout << HEADER_PRINT "Initial joint position arrived!" << endl;

    /* Publisher.*/
	out_joint_command_pub = nh_public.advertise<trajectory_msgs::JointTrajectory>(out_joint_command_topic_str, 1);

    /*Server.*/
    ros::ServiceServer serviceGetJoints = nh_public.advertiseService( service_get_joints_str, getJoints);

    //Initialize joint command
    out_jointTraj_msg.points[0].positions[0] = joint_state_msg.position[0];
    out_jointTraj_msg.points[0].positions[1] = joint_state_msg.position[1];
    out_jointTraj_msg.points[0].positions[2] = joint_state_msg.position[2];
    out_jointTraj_msg.points[0].positions[3] = joint_state_msg.position[3];
    out_jointTraj_msg.points[0].positions[4] = joint_state_msg.position[4];
    out_jointTraj_msg.points[0].positions[5] = joint_state_msg.position[5];
    out_jointTraj_msg.points[0].positions[6] = joint_state_msg.position[6];

    out_jointTraj_msg.points[0].velocities[0] = 0.0;
    out_jointTraj_msg.points[0].velocities[1] = 0.0;
    out_jointTraj_msg.points[0].velocities[2] = 0.0;
    out_jointTraj_msg.points[0].velocities[3] = 0.0;
    out_jointTraj_msg.points[0].velocities[4] = 0.0;
    out_jointTraj_msg.points[0].velocities[5] = 0.0;
    out_jointTraj_msg.points[0].velocities[6] = 0.0;


    ros::Rate loop_rate(COMMAND_RATE);
    double initial_time = ros::Time::now().toSec();
    while(ros::ok()){
        ros::spinOnce();

        out_jointTraj_msg.points[0].time_from_start = ros::Duration(ros::Time::now().toSec() - initial_time);
        out_joint_command_pub.publish(out_jointTraj_msg);

        loop_rate.sleep();
    }

    cout << HEADER_PRINT BOLDRED "END!" CRESET << endl;
    
    return 0;
}
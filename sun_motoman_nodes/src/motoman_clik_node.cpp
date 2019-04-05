#include "ros/ros.h"
#include "Robots/MotomanSIA5F.h"
#include "sun_robot_ros/CLIK_Node.h"
#include "motoman_interface/JointPositionVelocity.h"
#include "motoman_interface/GetJoints.h"

using namespace std;
using namespace TooN;

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


/*GLOBAL ROS VARS*/
ros::Publisher pub_joints;
ros::ServiceClient serviceGetJoint;
/*END Global ROS Vars*/

Vector<> getJointPosition_fcn(){
    
    motoman_interface::GetJoints getJoint_msg;

    bool b_result = serviceGetJoint.call(getJoint_msg);
    b_result = b_result && getJoint_msg.response.success;

    if(!b_result){
        cout << HEADER_PRINT << BOLDRED "ERROR! Unable to get joint_position!" CRESET << endl;
        exit(-1);
    }

    Vector<> out = Zeros(7);

    out[0] = getJoint_msg.response.position.s;
    out[1] = getJoint_msg.response.position.l;
    out[2] = getJoint_msg.response.position.e;
    out[3] = getJoint_msg.response.position.u;
    out[4] = getJoint_msg.response.position.r;
    out[5] = getJoint_msg.response.position.b;
    out[6] = getJoint_msg.response.position.t;

    return out;

}

void publish_fcn( const Vector<>& qR, const Vector<>& dqR ){
    motoman_interface::JointPositionVelocity out_msg;

    out_msg.position.s = qR[0];
    out_msg.position.l = qR[1];
    out_msg.position.e = qR[2];
    out_msg.position.u = qR[3];
    out_msg.position.r = qR[4];
    out_msg.position.b = qR[5];
    out_msg.position.t = qR[6];

    out_msg.velocity.s = dqR[0];
    out_msg.velocity.l = dqR[1];
    out_msg.velocity.e = dqR[2];
    out_msg.velocity.u = dqR[3];
    out_msg.velocity.r = dqR[4];
    out_msg.velocity.b = dqR[5];
    out_msg.velocity.t = dqR[6];

    out_msg.header.frame_id = "SIA5F";
    out_msg.header.stamp = ros::Time::now();

    pub_joints.publish(out_msg);
}

int main(int argc, char *argv[])
{

    ros::init(argc,argv, "motoman_clik");

    ros::NodeHandle nh_private = ros::NodeHandle("~");
    ros::NodeHandle nh_public = ros::NodeHandle();

    //params
    string topic_joint_command_str;
    nh_private.param("joint_command_topic" , topic_joint_command_str, string("simple_joint_command") );
    string service_get_joints_str;
    nh_private.param("service_get_joints" , service_get_joints_str, string("getJoints") );
    
    //Publishers
    pub_joints = nh_public.advertise<motoman_interface::JointPositionVelocity>(topic_joint_command_str, 1);

    //Service
    serviceGetJoint = nh_public.serviceClient<motoman_interface::GetJoints>(service_get_joints_str);

    cout << HEADER_PRINT "Wait for service getJoint Existence..." << endl;
    serviceGetJoint.waitForExistence();
    cout << HEADER_PRINT GREEN "Service getJoint online!" CRESET << endl;
    
    CLIK_Node clik_node(
            MotomanSIA5F( "SIA5F" ),
            nh_public,
            nh_private,
            getJointPosition_fcn, 
            publish_fcn
            );

    clik_node.run();

    return 0;
}
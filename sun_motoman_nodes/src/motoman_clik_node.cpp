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
    string desired_pose_twist_topic_str;
    nh_private.param("desired_pose_twist_topic" , desired_pose_twist_topic_str, string("desired_pose_twist") );
    string set_stopped_service;
    nh_private.param("set_stopped_service" , set_stopped_service, string("set_stopped") );
    double clik_gain;
    nh_private.param("clik_gain" , clik_gain, 0.5 );
    double dls_joint_speed_saturation;
    nh_private.param("dls_joint_speed_saturation" , dls_joint_speed_saturation, 3.0 );
    double second_obj_gain;
    nh_private.param("second_obj_gain" , second_obj_gain, 0.0 );
    bool start_stopped;
    nh_private.param("start_stopped" , start_stopped, true );

    std::vector<double> joint_target_std;
    nh_private.getParam("joint_target", joint_target_std);
    Vector<> joint_target = wrapVector(joint_target_std.data(), joint_target_std.size());

    std::vector<double> joint_weights_std;
    nh_private.getParam("joint_weights", joint_weights_std);
    Vector<> joint_weights = wrapVector(joint_weights_std.data(), joint_weights_std.size());

    std::vector<int> mask_std;
    nh_private.getParam("mask", mask_std);
    Vector<6,int> mask = wrapVector(mask_std.data(), mask_std.size());

    std::vector<double> n_T_e_position_std;
    nh_private.getParam("n_T_e_position", n_T_e_position_std);
    Vector<3> n_T_e_position = wrapVector(n_T_e_position_std.data(), n_T_e_position_std.size());

    std::vector<double> n_T_e_quaternion_std;
    nh_private.getParam("n_T_e_quaternion", n_T_e_quaternion_std);
    UnitQuaternion n_T_e_quaternion( wrapVector(n_T_e_quaternion_std.data(), n_T_e_quaternion_std.size()) );

    //Publishers
    pub_joints = nh_public.advertise<motoman_interface::JointPositionVelocity>(topic_joint_command_str, 1);

    //Service
    serviceGetJoint = nh_public.serviceClient<motoman_interface::GetJoints>(service_get_joints_str);

    Matrix<4,4> n_T_e = transl(n_T_e_position);
    n_T_e.slice<0,0,3,3>() = n_T_e_quaternion.torot();
    
    CLIK_Node clik_node(
            MotomanSIA5F( n_T_e, dls_joint_speed_saturation, "SIA5F" ), 
            nh_public,
            desired_pose_twist_topic_str, 
            set_stopped_service,
            getJointPosition_fcn, 
            publish_fcn,
            clik_gain,
            second_obj_gain,
            joint_target,
            joint_weights,
            mask,
            start_stopped
            );

    clik_node.run();

    return 0;
}
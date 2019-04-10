#include "ros/ros.h"
#include "Robots/MotomanSIA5F.h"
#include "sun_robot_ros/CLIK_Node.h"
#include "motoman_interface/JointPositionVelocity.h"
#include "motoman_interface/GetJoints.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

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
ros::Publisher pub_pose;
ros::ServiceClient serviceGetJoint;
/*END Global ROS Vars*/

CLIK_Node* clik_node;

Vector<7> qR;
void readJointState( const sensor_msgs::JointState::ConstPtr& joi_state_msg ){

    for(int i=0; i<7; i++)
        qR[i] = joi_state_msg->position[i];

    Matrix<4,4> b_T_e = clik_node->getRobot()->fkine( clik_node->getRobot()->joints_Robot2DH(qR) );

    UnitQuaternion uq(b_T_e);
    Vector<3> p = transl(b_T_e);

    geometry_msgs::PoseStamped out_msg;

    out_msg.header = joi_state_msg->header;
    out_msg.pose.position.x = p[0];
    out_msg.pose.position.y = p[1];
    out_msg.pose.position.z = p[2];
    out_msg.pose.orientation.w = uq.getS();
    Vector<3> uq_v = uq.getV();
    out_msg.pose.orientation.x = uq_v[0];
    out_msg.pose.orientation.y = uq_v[1];
    out_msg.pose.orientation.z = uq_v[2];

    pub_pose.publish(out_msg);
    
}

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
    string topic_ee_pose_str;
    nh_private.param("ee_pose_topic" , topic_ee_pose_str, string("ee_pose") );
    string service_get_joints_str;
    nh_private.param("service_get_joints" , service_get_joints_str, string("getJoints") );
    string joint_state_topic_str;
    nh_private.param("joint_state_topic" , joint_state_topic_str, string("joint_states") );

    //Subscribers
    ros::Subscriber joint_position_sub = nh_public.subscribe(joint_state_topic_str, 1, readJointState);
    
    //Publishers
    pub_joints = nh_public.advertise<motoman_interface::JointPositionVelocity>(topic_joint_command_str, 1);
    pub_pose = nh_public.advertise<geometry_msgs::PoseStamped>(topic_ee_pose_str, 1);

    //Service
    serviceGetJoint = nh_public.serviceClient<motoman_interface::GetJoints>(service_get_joints_str);

    cout << HEADER_PRINT "Wait for service getJoint Existence..." << endl;
    serviceGetJoint.waitForExistence();
    cout << HEADER_PRINT GREEN "Service getJoint online!" CRESET << endl;
    
    clik_node = new CLIK_Node(
            MotomanSIA5F( "SIA5F" ),
            nh_public,
            nh_private,
            getJointPosition_fcn, 
            publish_fcn
            );

    clik_node->run();

    delete clik_node;

    return 0;
}
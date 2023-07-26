#ifndef CRTK_INTERFACE_H
#define CRTK_INTERFACE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <math/CTransform.h>

#include <afFramework.h>
#include <ambf_server/RosComBase.h>

using namespace chai3d;
using namespace std;

class CRTKInterface{
public:
    CRTKInterface(string a_namespace);
    ~CRTKInterface();

    void init(string a_namespace);

    // ROS related
    ros::NodeHandle* m_rosNode;

    // Callback functions
    void poseCallback(geometry_msgs::PoseStampedConstPtr);
    void jointStateCallback(sensor_msgs::JointStateConstPtr);

    // Query Command
    cTransform& measured_cp();
    vector<double> measured_jp();

    // Motion Command
    void servo_cp(cTransform &trans);
    void servo_cf(vector<double>& force);
    void servo_jp(vector<double>& q);
    void move_jp(vector<double>& q);

    int m_numJoints;

private:
    // Subscribers
    ros::Subscriber m_jointStateSub;
    ros::Subscriber m_poseSub;

    // Publishers
    ros::Publisher m_servoCPPub;
    ros::Publisher m_servoCFPub;
    ros::Publisher m_servoJPPub;
    ros::Publisher m_moveJPPub;
    
    cTransform m_measured_cp;
    vector<double> m_measured_jp;

    geometry_msgs::PoseStamped m_servo_cp;
    geometry_msgs::WrenchStamped m_servo_cf;
    sensor_msgs::JointState m_servo_jp;
    sensor_msgs::JointState m_move_jp;
};


#endif //CRTK_INTERFACE_H
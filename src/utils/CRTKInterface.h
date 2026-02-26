#ifndef CRTK_INTERFACE_H
#define CRTK_INTERFACE_H

#include <ambf_server/ambf_ral.h>
#include <ambf_server/ambf_ral_config.h>
#include <ambf_server/RosComBase.h>
#include <afFramework.h>

#include <math/CTransform.h>

#if AMBF_ROS1
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#elif AMBF_ROS2
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#endif


using namespace chai3d;
using namespace std;

class CRTKInterface{
public:
    CRTKInterface(string a_namespace);
    ~CRTKInterface();

    void init(string a_namespace);

    // ROS related
    ambf_ral::node_ptr_t m_rosNode;

    // Callback functions
    void poseCallback(AMBF_RAL_MSG_PTR(geometry_msgs, PoseStamped));
    void jointStateCallback(AMBF_RAL_MSG_PTR(sensor_msgs, JointState));
    void forceCallback(AMBF_RAL_MSG_PTR(geometry_msgs, WrenchStamped));


    // Query Command
    cTransform& measured_cp();
    cVector3d& measured_cf();
    vector<double> measured_jp();

    // Motion Command
    void servo_cp(cTransform &trans);
    void servo_cf(vector<double>& force);
    void servo_jp(vector<double>& q);
    void move_jp(vector<double>& q);

    void spin();

    int m_numJoints;

private:
#if AMBF_ROS1
    // Subscribers
    ros::Subscriber m_jointStateSub;
    ros::Subscriber m_forceSub;
    ros::Subscriber m_poseSub;

    // Publishers
    ros::Publisher m_servoCPPub;
    ros::Publisher m_servoCFPub;
    ros::Publisher m_servoJPPub;
    ros::Publisher m_moveJPPub;

    geometry_msgs::PoseStamped m_servo_cp;
    geometry_msgs::WrenchStamped m_servo_cf;
    sensor_msgs::JointState m_servo_jp;
    sensor_msgs::JointState m_move_jp;
#elif AMBF_ROS2
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_jointStateSub;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr m_forceSub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_poseSub;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_servoCPPub;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr m_servoCFPub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_servoJPPub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_moveJPPub;

    geometry_msgs::msg::PoseStamped m_servo_cp;
    geometry_msgs::msg::WrenchStamped m_servo_cf;
    sensor_msgs::msg::JointState m_servo_jp;
    sensor_msgs::msg::JointState m_move_jp;
#endif

    cTransform m_measured_cp;
    cVector3d m_measured_cf;
    vector<double> m_measured_jp;

};


#endif //CRTK_INTERFACE_H
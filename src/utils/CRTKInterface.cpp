#include "CRTKInterface.h"

CRTKInterface::CRTKInterface(string a_namespace){
    init(a_namespace);
    m_numJoints = 6;
}

CRTKInterface::~CRTKInterface(){
    m_poseSub.shutdown();
    m_servoJPPub.shutdown();
    
    m_poseSub.shutdown();
    m_servoJPPub.shutdown();
}

void CRTKInterface::init(string a_namespace){
    m_rosNode = afROSNode::getNode();
    string baseName = a_namespace;
    cout << "Base Name:" << baseName << endl;
    m_poseSub = m_rosNode->subscribe(baseName + "/measured_cp", 1, &CRTKInterface::poseCallback, this);
    m_jointStateSub = m_rosNode->subscribe(baseName + "/measured_js", 1, &CRTKInterface::jointStateCallback, this);
    m_servoCPPub = m_rosNode->advertise<geometry_msgs::TransformStamped>(baseName + "/servo_cp", 1);
    m_servoCFPub = m_rosNode->advertise<geometry_msgs::WrenchStamped>(baseName + "/compliance/servo_cf", 1);
    m_servoJPPub = m_rosNode->advertise<sensor_msgs::JointState>(baseName+ "/servo_jp", 1);
    m_moveJPPub = m_rosNode->advertise<sensor_msgs::JointState>(baseName+ "/move_jp", 1);
}
        
    
void CRTKInterface::poseCallback(geometry_msgs::PoseStampedConstPtr msg){
    m_measured_cp.setLocalPos(cVector3d(msg->pose.position.x,
                                        msg->pose.position.y,
                                        msg->pose.position.z));
    cQuaternion rot(msg->pose.orientation.w,
                    msg->pose.orientation.x,
                    msg->pose.orientation.y,
                    msg->pose.orientation.z);
    cMatrix3d rotM;
    rot.toRotMat(rotM);

    m_measured_cp.setLocalRot(rotM);
}

void CRTKInterface::jointStateCallback(sensor_msgs::JointStateConstPtr msg){
    m_measured_jp = msg->position;

}

cTransform& CRTKInterface::measured_cp(){
    return m_measured_cp;
}

vector<double> CRTKInterface::measured_jp(){
    return m_measured_jp;
}

void CRTKInterface::servo_cp(cTransform &trans){
    m_servo_cp.pose.position.x = trans.getLocalPos().x();
    m_servo_cp.pose.position.y = trans.getLocalPos().y();
    m_servo_cp.pose.position.z = trans.getLocalPos().z();

    cQuaternion rot;
    rot.fromRotMat(trans.getLocalRot());
    m_servo_cp.pose.orientation.x = rot.x;
    m_servo_cp.pose.orientation.y = rot.y;
    m_servo_cp.pose.orientation.z = rot.z;
    m_servo_cp.pose.orientation.w = rot.w;

    m_servoCPPub.publish(m_servo_cp);
}

void CRTKInterface::servo_cf(vector<double>& force){

    if (force.size() != 6){
        cerr << "ERROR! IN SERVO_CF, FORCE HAS TO HAVE 6DOF." << endl;
        return;
    }

    m_servo_cf.wrench.force.x = force[0];
    m_servo_cf.wrench.force.y = force[1];
    m_servo_cf.wrench.force.z = force[2];
    m_servo_cf.wrench.torque.x = force[3];
    m_servo_cf.wrench.torque.y = force[4];
    m_servo_cf.wrench.torque.z = force[5];

    m_servoCFPub.publish(m_servo_cf);

}

void CRTKInterface::servo_jp(vector<double>& q){
    if (q.size() > m_numJoints){
        cerr << "ERROR! IN SERVO JP, JOINT LENGTH MUST BE GREATER THAN "<< m_numJoints << endl;
        return;
    }
    
    // for (int idx = 0 ; idx < q.size() ; idx++){
    //     // m_servo_jp.position[idx] = q[idx];
    // }
    vector<string> name = {"1", "2","3","4","5"};
    m_servo_jp.name = name;
    m_servo_jp.position = q;
    m_servoJPPub.publish(m_servo_jp);
}



void CRTKInterface::move_jp(vector<double>& q){
    if (q.size() >  m_numJoints){
        cerr << "ERROR! IN MOVE JP, JOINT LENGTH MUST BE GREATER THAN " << m_numJoints << endl;
        return;
    }

    for (int idx = 0 ; idx < q.size() ; idx++){
        m_move_jp.position[idx] = q[idx];
    }
    m_moveJPPub.publish(m_move_jp);
}
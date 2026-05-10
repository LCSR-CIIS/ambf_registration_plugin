#include "CRTKInterface.h"

CRTKInterface::CRTKInterface(string a_namespace){
    init(a_namespace);
    m_numJoints = 6;
}

CRTKInterface::~CRTKInterface(){
    ambf_ral::publisher_shutdown(m_servoCPPub); 
    ambf_ral::publisher_shutdown(m_servoJPPub);
    ambf_ral::subscriber_shutdown(m_poseSub);
    ambf_ral::subscriber_shutdown(m_jointStateSub);
    ambf_ral::subscriber_shutdown(m_forceSub);
}

void CRTKInterface::init(string a_namespace){
    m_rosNode = afROSNode::getNodeAndRegister(a_namespace);
    string baseName = a_namespace;
    cout << "Base Name:" << baseName << endl;

    ambf_ral::create_subscriber<AMBF_RAL_MSG(geometry_msgs, PoseStamped), CRTKInterface>
      (m_poseSub, m_rosNode, baseName + "/measured_cp", 1, &CRTKInterface::poseCallback, this);
    ambf_ral::create_subscriber<AMBF_RAL_MSG(sensor_msgs, JointState), CRTKInterface>
      (m_jointStateSub, m_rosNode, baseName + "/measured_js", 1, &CRTKInterface::jointStateCallback, this);
    ambf_ral::create_subscriber<AMBF_RAL_MSG(geometry_msgs, WrenchStamped), CRTKInterface>
      (m_forceSub, m_rosNode, baseName + "/measured_cf", 1 , &CRTKInterface::forceCallback, this);
    
    ambf_ral::create_publisher<AMBF_RAL_MSG(geometry_msgs, PoseStamped)>
      (m_servoCPPub, m_rosNode, baseName + "/servo_cp", 1, false);
    ambf_ral::create_publisher<AMBF_RAL_MSG(geometry_msgs, WrenchStamped)>
      (m_servoCFPub, m_rosNode, baseName + "/compliance/servo_cf", 1, false);
    ambf_ral::create_publisher<AMBF_RAL_MSG(sensor_msgs, JointState)>
      (m_servoJPPub, m_rosNode, baseName + "/servo_jp", 1, false);
    ambf_ral::create_publisher<AMBF_RAL_MSG(sensor_msgs, JointState)>
      (m_moveJPPub, m_rosNode, baseName + "/move_jp", 1, false);
}
        
    
void CRTKInterface::poseCallback(AMBF_RAL_MSG_PTR(geometry_msgs, PoseStamped) msg){
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

void CRTKInterface::jointStateCallback(AMBF_RAL_MSG_PTR(sensor_msgs, JointState) msg){
    m_measured_jp = msg->position;

}

void CRTKInterface::forceCallback(AMBF_RAL_MSG_PTR(geometry_msgs, WrenchStamped) msg){
    m_measured_cf.set(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
}

cTransform& CRTKInterface::measured_cp(){
    return m_measured_cp;
}

vector<double> CRTKInterface::measured_jp(){
    return m_measured_jp;
}

cVector3d& CRTKInterface::measured_cf(){
    return m_measured_cf;
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

    m_servoCPPub->publish(m_servo_cp);
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

    m_servoCFPub->publish(m_servo_cf);
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
    m_servoJPPub->publish(m_servo_jp);
}



void CRTKInterface::move_jp(vector<double>& q){
    if (q.size() >  m_numJoints){
        cerr << "ERROR! IN MOVE JP, JOINT LENGTH MUST BE GREATER THAN " << m_numJoints << endl;
        return;
    }

    for (int idx = 0 ; idx < q.size() ; idx++){
        m_move_jp.position[idx] = q[idx];
    }
    m_moveJPPub->publish(m_move_jp);
}

void CRTKInterface::spin(){
    if (m_rosNode){
        ambf_ral::spin_some(m_rosNode);
    }
    else{
        cerr << "[ERROR!!] NO ROS NODE initialized!!!" << endl;
    }
}
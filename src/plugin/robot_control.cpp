#include "robot_control.h"

RobotControlPlugin::RobotControlPlugin(){
}

int RobotControlPlugin::init(const afModelPtr a_modelPtr, afModelAttribsPtr a_attribs){

    cout << "/*********************************************" << endl;
    cout << "/* ROBOT CONTROL PLUGIN USING CRTK" << endl;
    cout << "/*********************************************" << endl;

    m_modelPtr = a_modelPtr;
    m_worldPtr = m_modelPtr->getWorldPtr();     //Ptr to the simulation world

    /*Get Main Camera*/
    m_mainCamera = m_worldPtr->getCameras()[0];
    if(!m_mainCamera){
        m_mainCamera = m_modelPtr->getCameras()[0];
    }

    // For atracsys based control
    m_robotInterface = new CRTKInterface("/atracsys/Endoscope/local");
    m_referenceInterface = new CRTKInterface("/atracsys/Reference/local");

    m_dovetailPtr = m_modelPtr->getRigidBody("Marker");

    return 1;
}

void RobotControlPlugin::graphicsUpdate(){

}

void RobotControlPlugin::physicsUpdate(double dt){

    cTransform measured_cp = m_robotInterface->measured_cp();

    // For atracsys based control with reference frame
    cTransform ref_measured_cp = m_referenceInterface->measured_cp();
    ref_measured_cp.invert();
    measured_cp = ref_measured_cp * measured_cp;

    if (measured_cp.getData())
    {   // Convert Rotation into quaternion
        cQuaternion chai_qr;
        chai_qr.fromRotMat(measured_cp.getLocalRot());

        // Change to btVector and Matrix
        btTransform trans;
        btVector3 btTrans;
        btTrans.setValue(measured_cp.getLocalPos().x(), measured_cp.getLocalPos().y(), measured_cp.getLocalPos().z());   
        btQuaternion btRot(chai_qr.x,chai_qr.y, chai_qr.z, chai_qr.w);
        trans.setOrigin(btTrans);
        trans.setRotation(btRot);

        btTransform Tcommand;
        Tcommand = trans;
        m_dovetailPtr->m_bulletRigidBody->getMotionState()->setWorldTransform(Tcommand);
        m_dovetailPtr->m_bulletRigidBody->setWorldTransform(Tcommand);
    }

}



void RobotControlPlugin::reset(){

}

bool RobotControlPlugin::close(){
    delete m_robotInterface;
    return 0;
}

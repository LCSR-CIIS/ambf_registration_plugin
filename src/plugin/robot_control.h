#ifndef ROBOT_CONTROL_CONTROL_H
#define ROBOT_CONTROL_CONTROL_H


#include "../utils/CRTKInterface.h"
#include <afFramework.h>
#include <algorithm>
#include <cmath>


using namespace ambf;
using namespace std;

class RobotControlPlugin: public afModelPlugin {

public:
    RobotControlPlugin();
    virtual int init(const afModelPtr a_modelPtr, afModelAttribsPtr a_attribs) override;

    virtual void graphicsUpdate() override;
    virtual void physicsUpdate(double dt) override;

    virtual void reset() override;
    virtual bool close() override;

    CRTKInterface* m_robotInterface;
    CRTKInterface* m_referenceInterface;

// private:


    afModelPtr m_modelPtr;
    afWorldPtr m_worldPtr;


    // camera to render the world
    afCameraPtr m_mainCamera;

    afRigidBodyPtr m_dovetailPtr;

};

AF_REGISTER_MODEL_PLUGIN(RobotControlPlugin)

#endif //ROBOT_CONTROL_PLUGIN_H

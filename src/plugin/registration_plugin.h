//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2023, AMBF
    (https://github.com/WPI-AIM/ambf)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <hishida3@jhu.edu>
    \author    Hisashi Ishida
*/
//==============================================================================
// To silence warnings on MacOS
#define GL_SILENCE_DEPRECATION
#include <afFramework.h>

#include "../utils/camera_panel_manager.h"
#include "../utils/CRTKInterface.h"
#include <yaml-cpp/yaml.h>

#include "../registration/point_cloud_registration.h"
#include "../registration/hand_eye_calibration.h"
#include "../registration/pivot_calibration.h"


#include <boost/program_options.hpp>

#include <fstream>
#include <ctime>

namespace boost{
    namespace program_options{
        class variables_map;
    }
}

namespace p_opt = boost::program_options;

using namespace std;
using namespace ambf;

enum class RegistrationMode{
    UNREGISTERED=0, POINTER=1, TRACKER=2, PIVOT=3, HANDEYE=4, REGISTERED=5 
};

class afRegistrationPlugin: public afSimulatorPlugin{
    public:
        afRegistrationPlugin();
        virtual int init(int argc, char** argv, const afWorldPtr a_afWorld) override;
        virtual void keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods) override;
        virtual void graphicsUpdate() override;
        virtual void physicsUpdate(double dt) override;
        virtual void reset() override;
        virtual bool close() override;

    protected:
        bool initLabels();
        bool initCamera(vector<string> cameraNames);
        int readConfigFile(string config_filepath);

        void addOptionDescription(string & text);

    // private:
        // Pointer to the world
        afWorldPtr m_worldPtr;

        // Path
        string m_current_filepath;

        // camera related
        map<string, afCameraPtr> m_cameras;

        // Pop-up Panel related
        CameraPanelManager m_panelManager;
        cLabel* m_registrationStatusLabel;
        cLabel* m_savedPointsListLabel;
        bool m_enableList = true;
        // string m_savedLocationText;


        RegistrationMode m_activeMode = RegistrationMode::UNREGISTERED;

        // Pointer based registration
        afRigidBodyPtr m_toolTipPtr = nullptr;
        int m_numPoints = 0;
        int m_numTrackingPoints = 0;

        vector<afRigidBodyPtr> m_pointsPtr; 
        vector<afRigidBodyPtr> m_trackingPointsPtr; 
        
        // Pointer based Registration
        bool m_savePoint = false;
        vector<cShapeSphere*> m_spheres;
        vector<cVector3d> m_savedPositions;
        vector<cVector3d> m_savedError;

        vector<cVector3d> m_pointsIn;
        vector<cVector3d> m_pointsOut;
        cTransform m_registerdTrans;

        // Tracker based Registration
        vector<CRTKInterface*> m_trackingInterfaces;
        vector<cTransform> m_savedAMBFPoints;
        vector<cTransform> m_savedRef2Points;
        bool m_flagTrack = false;

        // Registered Statistics Text
        string m_registeredText;
        btTransform m_registeredTransform;

        afRigidBodyPtr m_registeringObject;
        vector<cVector3d> m_result;
       
        // Point cloud registration
        PointCloudRegistration m_pointCloudRegistration;


        // Hand-eye Registration
        HandEyeCalibration m_handEyeCalibration;
        CRTKInterface * m_robotInterface;
        CRTKInterface * m_toolInterface;
        afRigidBodyPtr m_eeJointPtr;
        afRigidBodyPtr m_markerPtr;
        vector<cTransform> m_savedRobotPoints;
        cTransform m_ee2marker;
        btTransform m_btee2marker;
        cTransform m_tracker;
        double m_trackRes = 0.001;
        int m_numHE = 1000;
        bool m_flagHE = false;
        
        // Pivot-calibration
        PivotCalibration m_pivotCalibration;
        vector<cTransform> m_savedPivotPoints;
        double m_pivotRes = 0.001;
        int m_numPivot = 1000;
        bool m_flagPivot = false;

        cVector3d m_marker2tip;

        cShapeSphere* m_burrMesh;
        bool m_robotPivot = false;

        cVector3d m_ee2tip;


        // These two pointers are for optional Objects
        afRigidBodyPtr m_toolPtr; 
        afRigidBodyPtr m_trackerPtr;

        vector<cTransform> m_savedPoints;

        // Flag used to check if the mode was defined in the config file.
        bool m_isPivot = false;
        bool m_isHE = false; 
        bool m_isPointer = false;
        bool m_isOT = false;

};


void saveDataToCSV(string fileName, vector<cTransform> vecTransform);

AF_REGISTER_SIMULATOR_PLUGIN(afRegistrationPlugin)

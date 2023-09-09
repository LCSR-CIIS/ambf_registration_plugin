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
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN        DIRECT,
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

#include "registration_plugin.h"

afRegistrationPlugin::afRegistrationPlugin(){
    cout << "/*********************************************" << endl;
    cout << "/* AMBF Plugin for Registration" << endl;
    cout << "/*********************************************" << endl;
}

int afRegistrationPlugin::init(int argc, char** argv, const afWorldPtr a_afWorld){
    p_opt::options_description cmd_opts("SpaceNav control Command Line Options");
    cmd_opts.add_options()
            ("info", "Show Info")
            ("registration_config", p_opt::value<string>()->default_value(""), "Name of specfile for spacenav control");

    p_opt::variables_map var_map;
    p_opt::store(p_opt::command_line_parser(argc, argv).options(cmd_opts).allow_unregistered().run(), var_map);
    p_opt::notify(var_map);

    if(var_map.count("info")){
        std::cout<< cmd_opts << std::endl;
        return -1;
    }

    // Loading options 
    string config_filepath = var_map["registration_config"].as<string>();

    // Define path
    string file_path = __FILE__;
    m_current_filepath = file_path.substr(0, file_path.rfind("/"));

    // Initialize Camera
    m_worldPtr = a_afWorld;

    // Improve the constratint
    m_worldPtr->m_bulletWorld->getSolverInfo().m_erp = 1.0;  // improve out of plane error of joints
    m_worldPtr->m_bulletWorld->getSolverInfo().m_erp2 = 1.0; // improve out of plane error of joints

    vector<string> cameraNames = {"main_camera", "cameraL", "cameraR", "stereoLR"};
    bool  initcam = initCamera(cameraNames);
    
    if (initcam && initLabels()){
        cerr << "SUCCESSFULLY Initialize Camera and Labels" << endl;
    }
    
    else{
        cerr << "ERROR! FAILED To Initialize Camera and/or Lables" << endl;
        return -1;
    }


    // When config file was defined
    if(!config_filepath.empty()){
        return readConfigFile(config_filepath);
    }

    else{
        cerr << "ERROR! NO configuration file specified." << endl;
        return -1;
    }
}

bool afRegistrationPlugin::initLabels(){
    // create a font
    cFontPtr font = NEW_CFONTCALIBRI20();

    // Active Object panel
    m_registrationStatusLabel = new cLabel(font);
    m_registrationStatusLabel->m_fontColor.setRedCrimson();
    m_registrationStatusLabel->setCornerRadius(5, 5, 5, 5);
    m_registrationStatusLabel->setShowPanel(true);
    m_registrationStatusLabel->setColor(cColorf(1.0, 1.0, 1.0, 1.0));

    m_panelManager.addPanel(m_registrationStatusLabel, 0.8, 0.9, PanelReferenceOrigin::LOWER_LEFT, PanelReferenceType::NORMALIZED);
    m_panelManager.setVisible(m_registrationStatusLabel, true);

    // Objects List panel
    m_savedPointsListLabel = new cLabel(font);
    m_savedPointsListLabel->setFontScale(0.8);
    m_savedPointsListLabel->m_fontColor.setBlack();
    m_savedPointsListLabel->setCornerRadius(5, 5, 5, 5);
    m_savedPointsListLabel->setShowPanel(true);
    m_savedPointsListLabel->setColor(cColorf(1.0, 1.0, 1.0, 1.0));

    m_panelManager.addPanel(m_savedPointsListLabel, 0.8, 0.8, PanelReferenceOrigin::LOWER_LEFT, PanelReferenceType::NORMALIZED);
    m_panelManager.setVisible(m_savedPointsListLabel, true);

    return true;

}
bool afRegistrationPlugin::initCamera(vector<string> cameraNames){
    cout << "> Initializing CAMERA ..." << endl;
    if (cameraNames.size() == 0){
        cerr << "ERROR! NO CAMERA Specified." << endl;
        return false;
    }
    

    for (int i = 0 ; i < cameraNames.size() ; i++){
        afCameraPtr cam = m_worldPtr->getCamera(cameraNames[i]);
        if (cam){
            cerr << "INFO! GOT CAMERA: " << cam->getName() << endl;
            m_cameras[cameraNames[i]] = cam;
        }
        // if there is no main_camera load the first camera from the world.
        if(cameraNames[i] == "main_camera" && !cam){
            cerr << "INFO! FAILED TO LOAD main_camera, taking the first camera from world " << endl;
            cam = m_worldPtr->getCameras()[0];
            m_cameras["main_camera"] = cam;
        }
    } 

    // Specify the camera for the text overlays
    m_panelManager.addCamera(m_cameras["main_camera"]);
    if (m_cameras["steroLR"]){
        m_cameras["steroLR"]->getInternalCamera()->m_stereoOffsetW = 0.1;
        m_panelManager.addCamera(m_cameras["main_camera"]);
    }

    cBackground* background = new cBackground();
    background->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(0.6f, 0.6f, 0.6f),
                                cColorf(0.6f, 0.6f, 0.6f));
    m_cameras["main_camera"]->getBackLayer()->addChild(background);

    return true;
}


void afRegistrationPlugin::keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods){
    if (a_mods == GLFW_MOD_CONTROL){
        if (a_key == GLFW_KEY_1){
            m_activeMode = RegistrationMode::HANDEYE;
            cerr << "Registration Mode changed to HANDEYE " << endl;
        }

        else if (a_key == GLFW_KEY_2){
            m_activeMode = RegistrationMode::PIVOT;
            cerr << "Registration Mode changed to PIVOT " << endl;
        }
        
        else if (a_key == GLFW_KEY_3){
            m_activeMode = RegistrationMode::POINTER;
            cerr << "Registration Mode changed to POINTER " << endl;
        }

        else if (a_key == GLFW_KEY_4){
            m_activeMode = RegistrationMode::TRACKER;
            cerr << "Registration Mode changed to TRACKER " << endl;
        }
    
        else if(a_key == GLFW_KEY_9){
            if (m_activeMode == RegistrationMode::POINTER){
                m_savePoint = true;
                cerr << "Saving Tooltip location ..." << endl;
            }
        }
    }
}

void afRegistrationPlugin::graphicsUpdate(){

    string m_savedLocationText = "--- List of Saved Locations --- \n";

    // For Registration Status Label
    switch (m_activeMode){
        case RegistrationMode::UNREGISTERED:
            m_panelManager.setText(m_registrationStatusLabel, "Registration Status: UNREGISTER");
            m_panelManager.setFontColor(m_registrationStatusLabel, cColorf(1.,0.,0.));
            break;
        
        case RegistrationMode::POINTER:
            m_panelManager.setText(m_registrationStatusLabel, "Registration Status: POINTER");
            m_panelManager.setFontColor(m_registrationStatusLabel, cColorf(1.,0.,0.));

            // Saving the saved points location as text and show on the screen.
            for (int i=0; i < m_spheres.size(); i++){   
                cVector3d trans = m_spheres[i]->getLocalPos();
                m_savedLocationText += "Point " + to_string(i) + ": " + trans.str(5);
                if(i < m_spheres.size()-1){
                    m_savedLocationText += "\n";
                }
            }
            m_panelManager.setText(m_savedPointsListLabel, m_savedLocationText);
            break;

        case RegistrationMode::TRACKER:
            m_panelManager.setText(m_registrationStatusLabel, "Registration Status: TRACKER");
            m_panelManager.setFontColor(m_registrationStatusLabel, cColorf(1.,0.,0.));
            m_panelManager.setText(m_savedPointsListLabel, m_registeredText);

            break;

        case RegistrationMode::PIVOT:
            m_panelManager.setText(m_registrationStatusLabel, "Registration Status: PIVOT");
            m_panelManager.setFontColor(m_registrationStatusLabel, cColorf(1.,0.,0.));
            m_panelManager.setText(m_savedPointsListLabel, m_registeredText);
            m_panelManager.setFontColor(m_savedPointsListLabel, cColorf(0.,0.,0.));

            break;

        case RegistrationMode::HANDEYE:
            m_panelManager.setText(m_registrationStatusLabel, "Registration Status: HANDEYE");
            m_panelManager.setText(m_savedPointsListLabel, m_registeredText);
            m_panelManager.setFontColor(m_registrationStatusLabel, cColorf(1.,0.,0.));
            m_panelManager.setFontColor(m_savedPointsListLabel, cColorf(0.,0.,0.));
            break;

        case RegistrationMode::REGISTERED:
            m_panelManager.setText(m_registrationStatusLabel, "Registration Status: REGISTERED");
            m_panelManager.setFontColor(m_registrationStatusLabel, cColorf(0.,0.5,0.5));

            m_panelManager.setText(m_savedPointsListLabel, m_registeredText);
            break;
    }

    m_panelManager.update();

}

void afRegistrationPlugin::physicsUpdate(double dt){
    if (m_activeMode == RegistrationMode::POINTER){
        if (m_savePoint){
            cShapeSphere* pointMesh = new cShapeSphere(0.001);
            pointMesh->setRadius(0.001);
            pointMesh->m_material->setRed();
            pointMesh->m_material->setShininess(0);
            pointMesh->m_material->m_specular.set(0, 0, 0);
            pointMesh->setShowEnabled(true);
            pointMesh->setLocalPos(m_toolTipPtr->getLocalPos());

            // Using btvector
            // btVector3 tip = m_toolTipPtr->m_bulletRigidBody->getCenterOfMassPosition();
            // pointMesh->setLocalPos(cVector3d(tip.x(), tip.y(), tip.z()));
            m_worldPtr->addSceneObjectToWorld(pointMesh);
            m_spheres.push_back(pointMesh);

            m_savePoint = false;
        }

        // TODO: Add/Remove saved Points if needed
        // Once all the points are saved
        if (m_spheres.size() == m_pointsPtr.size()){
            cerr << "Saving all the translational error." << endl;
            for (int idx=0; idx<m_spheres.size(); idx++){
                m_savedError.push_back(m_pointsPtr[idx]->getLocalPos() - m_spheres[idx]->getLocalPos());
                m_pointsIn.push_back(m_pointsPtr[idx]->getLocalPos());
                m_pointsOut.push_back(m_spheres[idx]->getLocalPos());
            }

            // Perform ICP registration
            // bool resultPCRegist = m_pointCloudRegistration.ICPRegistration(m_pointsIn, m_pointsOut, m_registeredTransform);

            // Perform Point set Registration
            vector<cVector3d> newPoints;
            bool resultPCRegist = m_pointCloudRegistration.PointSetRegistration(m_pointsIn, m_pointsOut, m_registeredTransform, newPoints);

            for (size_t i = 0; i < newPoints.size(); i++){
                cerr << "Setting the points ..." << endl;   
                cShapeSphere* point = new cShapeSphere(0.001);
                point->setRadius(0.001);
                point->m_material->setBlue();
                point->m_material->setShininess(0);
                point->m_material->m_specular.set(0, 0, 0);
                point->setShowEnabled(true);
                point->setLocalPos(newPoints[i]);
                m_worldPtr->addSceneObjectToWorld(point);
            }

            if (resultPCRegist){
                // Change mode to "REGISTERED"
                // btTransform Tcommand;

                // btTransform currentTrans = m_registeringObject->m_bulletRigidBody->getWorldTransform();
                // Tcommand =  currentTrans * m_registeredTransform;
                // m_registeringObject->m_bulletRigidBody->getMotionState()->setWorldTransform(Tcommand);
                // m_registeringObject->m_bulletRigidBody->setWorldTransform(Tcommand);

                // btScalar x, y, z;
                // m_registeringObject->m_bulletRigidBody->getWorldTransform().getBasis().getEulerZYX(z, y, x);

                // cerr << "Roll: " << x << "," << "Pitch: " << y << "," << "Yaw: " << z << endl; 

                m_activeMode = RegistrationMode::REGISTERED;
            }

        }
            
    }

    else if (m_activeMode == RegistrationMode::TRACKER){
        cTransform measured_cp = m_toolInterface->measured_cp();
        m_registeredText = "WARNING! No tool location published \nCheck your tracker!!\n";

        if (measured_cp.getLocalPos().length() > 0.0){
            m_registeredText = "";
        }
        
        cVector3d measured_cf = m_robotInterface->measured_cf();

        m_registeredText += "WARNING! No robot related topic published";
        
        // ToDo: Change this to Robot Mode
        if(measured_cp.getLocalPos().length() > 0.0 && !m_flagTrack){
            // Erase the warning if there is measured_cf
            m_registeredText = "Saving Points...\n";

            // get trackerLocation data from rostopics
            cTransform collectedPoint = m_toolInterface->measured_cp();
            cTransform collectedReference = m_trackingInterfaces[0]->measured_cp();
            collectedReference.invert();

            if (m_savedPoints.size() == 0){
                m_savedPoints.push_back(collectedPoint);
                m_savedRef2Points.push_back(collectedReference * collectedPoint);
                m_savedAMBFPoints.push_back(m_eeJointPtr->getLocalTransform());
                m_savedRobotPoints.push_back(m_robotInterface->measured_cp());
            }
            else {
                // Save only the new collected points which are far enough from old points
                if ((m_savedPoints.back().getLocalPos() - collectedPoint.getLocalPos()).length() > m_trackRes){
                    m_savedPoints.push_back(collectedPoint);
                    m_savedRef2Points.push_back(collectedReference * collectedPoint);
                    m_savedAMBFPoints.push_back(m_eeJointPtr->getLocalTransform());
                    m_savedRobotPoints.push_back(m_robotInterface->measured_cp());
                }
            }

            // Once you collected enough points for the calibration
            if (m_savedPoints.size() > m_numHE && !m_flagTrack){
                m_registeredText = "Saving Points into csv file.";
                saveDataToCSV("HE_trackerTomarker.csv", m_savedPoints);             
                saveDataToCSV("HE_referenceTomarker.csv", m_savedRef2Points);             
                saveDataToCSV("HE_ambfToEE.csv", m_savedAMBFPoints);   
                saveDataToCSV("HE_worldToEE.csv", m_savedRobotPoints);   
                m_registeredText = "[INFO] Saved to /data/ folder!";
                cerr << "Saved to /data/ folder!" << endl;
                m_flagTrack = true;
            }

            else if(m_savedPoints.size() > 0){
                m_registeredText += "Number of saved Points: " + to_string(m_savedPoints.size()) + "/" + to_string(m_numHE);
            }

            if(m_flagTrack){
                m_registeredText = "Saved Tracking data!!";
            }
        }

    }

    else if (m_activeMode == RegistrationMode::HANDEYE){
        if(!m_flagHE){
            cTransform measured_cp = m_toolInterface->measured_cp();
            m_registeredText = "WARNING! No tool location published \nCheck your tracker!!\n";

            if (measured_cp.getLocalPos().length() > 0.0){
                m_registeredText = "";
            }

            cVector3d measured_cf = m_robotInterface->measured_cf();

            m_registeredText += "WARNING! No robot related topic published";
            
            // ToDo: Change this to Robot Mode
            if(measured_cf.length() < 10.0){
                // Erase the warning if there is measured_cf
                m_registeredText = "";

                // get trackerLocation data from rostopics
                cTransform collectedPoint = m_toolInterface->measured_cp();

                if (m_savedPoints.size() == 0){
                    m_savedPoints.push_back(collectedPoint);
                    m_savedRobotPoints.push_back(m_eeJointPtr->getLocalTransform());
                }
                else {
                    // Save only the new collected points which are far enough from old points
                    if ((m_savedPoints.back().getLocalPos() - collectedPoint.getLocalPos()).length() > m_trackRes){
                        m_savedPoints.push_back(collectedPoint);
                        m_savedRobotPoints.push_back(m_eeJointPtr->getLocalTransform());
                    }
                }

                // Once you collected enough points for the calibration
                if (m_savedPoints.size() > m_numHE){
                    cTransform ee2marker;
                    cTransform tracker;
                    // m_handEyeCalibration.calibrate(m_savedRobotPoints, m_savedPoints, m_ee2marker, m_tracker);
                    m_flagHE = true;
                }
            }        

            if (m_savedPoints.size() > 0){
                m_registeredText = "Number of saved Points: " + to_string(m_savedPoints.size());
            }
        }
    }
    
    else if (m_activeMode == RegistrationMode::PIVOT){
        cTransform measured_cp = m_toolInterface->measured_cp();
        m_registeredText = "WARNING! No tool location published \nCheck your tracker!!\n";

        if (measured_cp.getLocalPos().length() > 0.0 && !m_flagPivot){
            // Erase the warning if there is measured_cf
            m_registeredText = "Saving Points...\n";

            // get trackerLocation data from rostopics
            cTransform collectedPoint = m_toolInterface->measured_cp();
            cTransform collectedReference = m_trackingInterfaces[0]->measured_cp();
            collectedReference.invert();

            if (m_savedPivotPoints.size() == 0){
                m_savedPivotPoints.push_back(collectedPoint);
                m_savedRef2Points.push_back(collectedReference * collectedPoint);
            }
            else {
                // Save only the new collected points which are far enough from old points
                if ((m_savedPivotPoints.back().getLocalPos() - collectedPoint.getLocalPos()).length() > m_pivotRes){
                    m_savedPivotPoints.push_back(collectedPoint);
                    m_savedRef2Points.push_back(collectedReference * collectedPoint);
                }
            }

            // Once you collected enough points for the calibration
            if (m_savedPivotPoints.size() > m_numPivot){
                cVector3d dimple;
                cVector3d test;
                m_pivotCalibration.calibrate(m_savedPivotPoints, m_marker2tip, dimple);
                m_pivotCalibration.calibrate(m_savedRef2Points, test, dimple);

                // If you want to save the points
                if(1){
                    m_registeredText = "Saving Points into csv file.";
                    saveDataToCSV("Pivot_trackerTomarker.csv", m_savedPivotPoints);             
                    saveDataToCSV("Pivot_referenceTomarker.csv", m_savedRef2Points);             
                    m_registeredText = "[INFO] Saved to /data/ folder!";
                    cerr << "Saved to /data/ folder!" << endl;
                }
                
                m_flagPivot = true;
            }

            else if(m_savedPivotPoints.size() > 0){
                m_registeredText += "Number of saved Points: " + to_string(m_savedPivotPoints.size()) + "/" + to_string(m_numPivot);
            }
        }
    }
    
    else if (m_activeMode == RegistrationMode::REGISTERED){
        // Saving text for the status monitor
        m_registeredText = "Registeration Result: \n Avg: " + to_string(m_registeredTransform.getOrigin().x()) + "," +
        to_string(m_registeredTransform.getOrigin().y()) + "," + to_string(m_registeredTransform.getOrigin().z()) + "\n";
    }

    // Once you finish HandEye registration
    if (m_flagHE){
        if(m_markerPtr){
            // Move marker to using the HandEye calibration
            // Use cTransform to move
            // cTransform marker = m_eeJointPtr->getLocalTransform();
            // marker.mul(m_ee2marker);
            // m_markerPtr->setLocalTransform(marker);
            // m_burrMesh->setLocalPos(marker.getLocalPos());

            // // // Use btTransform to move the Marker
            // btTransform Tcommand;
            // btTransform currentTrans = m_eeJointPtr->m_bulletRigidBody->getWorldTransform();
            // m_btee2marker.inverse();
            // Tcommand.mult(currentTrans, m_btee2marker);

            // // // Use btTransform to move the Marker
            btTransform Tcommand;
            m_eeJointPtr->m_bulletRigidBody->getMotionState()->getWorldTransform(Tcommand);
            Tcommand.mult(Tcommand, m_btee2marker);
            m_markerPtr->m_bulletRigidBody->getMotionState()->setWorldTransform(Tcommand);
            m_markerPtr->m_bulletRigidBody->setWorldTransform(Tcommand);
        }
    }

    if (m_flagPivot){
        cVector3d tmp;
        m_ee2marker.mulr(m_marker2tip, tmp);
        cVector3d tip;
        m_eeJointPtr->getLocalTransform().mulr(tmp, tip);

        // Move the drill tip to the correct location.
        // cTransform result;
        // m_ee2marker.mulr(m_eeJointPtr->getLocalTransform(), result);
        // result.mulr(m_marker2tip, tip);

        // m_burrMesh->setLocalPos(tip);
        // m_toolTipPtr->setLocalPos(tip);
        // btTransform Tcommand;
        // Tcommand.setOrigin(btVector3(tip.x(), tip.y(), tip.z()));
        // btTransform current_ee;
        // m_eeJointPtr->m_bulletRigidBody->getMotionState()->getWorldTransform(current_ee);
        // Tcommand = current_ee * Tcommand;
        // m_toolTipPtr->m_bulletRigidBody->getMotionState()->setWorldTransform(Tcommand);
        // m_toolTipPtr->m_bulletRigidBody->setWorldTransform(Tcommand);
        // m_toolTipPtr->setLocalPos(tip);
    }

    if(m_flagHE && m_flagPivot){
        cVector3d finalTransform;
        m_ee2marker.mulr(m_marker2tip, finalTransform);
        m_registeredText += "EE2Tooltip: " + finalTransform.str(6);
        // cout << "Final Registration result: " << endl;
        // cout << "EE2Tooltip: "  << finalTransform.str(6) << endl;
    }
}

int afRegistrationPlugin::readConfigFile(string config_filepath){
        cerr << "> loading the user defined configuration file..." << endl; 
        cerr << config_filepath << endl;

        //Load the user defined object here. 
        YAML::Node node = YAML::LoadFile(config_filepath);
        
        // Check whether pointer based registration is needed or not
        if (node["pointer"]){
            cout << "Pointer based Registration" << endl;
            // Get the name of the tooltip object
            string toolTipName = node["pointer"]["tooltip name"].as<string>();

            m_toolTipPtr = m_worldPtr->getRigidBody(toolTipName);

            if(!m_toolTipPtr){
                cerr << "ERROR! NO Tooltip named: " << toolTipName << endl;
                return -1;
            }

            m_numPoints = node["pointer"]["name of points"].size();
            // Load alll the points
            for (int i=0; i < m_numPoints; i++){

                afRigidBodyPtr objectPtr;
                objectPtr = m_worldPtr->getRigidBody(node["pointer"]["name of points"][i].as<string>());
                
                if(objectPtr){
                    m_pointsPtr.push_back(objectPtr);
                }
                else{
                    cerr << "WARNING! No point named " << node["pointer"]["name of points"][i].as<string>() << " found." << endl;
                }
            }

            cerr << m_numPoints << "Points are specified as Keypoints" << endl;
            
            m_registeringObject = m_worldPtr->getRigidBody(node["pointer"]["object name"].as<string>());

            if(!m_registeringObject){
                cerr << "ERROR! No object named " << node["pointer"]["object name"].as<string>() << " found." << endl;
                
                return -1;
            }
        }

        // Check whether optical tracker based registration is needed or not
        if (node["optical tracker"]){
            cout << "Optical Tracker based registration" << endl;

            string nspace = node["optical tracker"]["namespace"].as<string>();
            m_numTrackingPoints = node["optical tracker"]["name of points"].size();
            
            // Load alll the points
            for (int i=0; i < m_numTrackingPoints; i++){

                afRigidBodyPtr objectPtr;
                string objectName = node["optical tracker"]["name of points"][i].as<string>();
                objectPtr = m_worldPtr->getRigidBody(objectName);
                
                if(1){
                // if(objectPtr){
                    m_trackingPointsPtr.push_back(objectPtr);
                    CRTKInterface* interface = new CRTKInterface(nspace + "/" + objectName);
                    m_trackingInterfaces.push_back(interface);
                }
                else{
                    cerr << "WARNING! No object named " << objectName << " found." << endl;
                }
            }
        }

        if (node["hand eye"]){
            cout << "Hand Eye Calibration" << endl;

            // Get marker in AMBF
            string markerName = node["hand eye"]["marker name"].as<string>();
            m_toolPtr = m_worldPtr->getRigidBody(markerName);
            
            if(!m_toolPtr){
                cerr << "WARNING! No marker named " << markerName << "found." << endl;
            }

            // Setting up CRTK communication
            string nspace = node["hand eye"]["namespace"].as<string>();
            m_robotInterface = new CRTKInterface(nspace);
            m_toolInterface = new CRTKInterface("/" + markerName + "/");

            // Get joint in AMBF
            string jointName = node["hand eye"]["joint name"].as<string>();
            m_eeJointPtr = m_worldPtr->getRigidBody(jointName);
            
            if(!m_eeJointPtr){
                cerr << "ERROR! No Endeffector pose named " << jointName << "found." << endl;
            }


            if(node["hand eye"]["optical tracker name"]){
                string trackerName = node["hand eye"]["optical tracker name"].as<string>();
                m_trackerPtr = m_worldPtr->getRigidBody(trackerName);

                if(!m_trackerPtr){
                cerr << "WARNING! No Tracker named " << trackerName << "found." << endl;
                }
            }

            if(node["hand eye"]["resolution"]){
                m_trackRes = node["hand eye"]["resolution"].as<double>();
            }

            if(node["hand eye"]["number of points"]){
                m_numHE = node["hand eye"]["number of points"].as<int>();
            }

            if(node["hand eye"]["registered HE result"]){
                double x = node["hand eye"]["registered HE result"]["q_rot"]["x"].as<double>();
                double y = node["hand eye"]["registered HE result"]["q_rot"]["y"].as<double>();
                double z = node["hand eye"]["registered HE result"]["q_rot"]["z"].as<double>();
                double w = node["hand eye"]["registered HE result"]["q_rot"]["w"].as<double>();
                cQuaternion q_rot(w, x, y, z);
                cerr << "Quaternion Rot: " << q_rot.str(5) << endl;
                // q_rot.invert();
                btQuaternion btRot(q_rot.x, q_rot.y, q_rot.z, q_rot.w);
                m_btee2marker.setRotation(btRot);  

                cMatrix3d rot;
                q_rot.toRotMat(rot);
                m_ee2marker.setLocalRot(rot);
                x = node["hand eye"]["registered HE result"]["q_dual"]["x"].as<double>();
                y = node["hand eye"]["registered HE result"]["q_dual"]["y"].as<double>();
                z = node["hand eye"]["registered HE result"]["q_dual"]["z"].as<double>();
                w = node["hand eye"]["registered HE result"]["q_dual"]["w"].as<double>();
                cQuaternion q_dual(w, x, y, z);
                q_rot.conj();
                q_dual.mul(q_rot);
                q_dual.mul(0.5);
                cerr << "Quaternion Trans: " << q_dual.str(5) << endl;
                btVector3 btTrans;
                btTrans.setValue(q_dual.x, q_dual.y, q_dual.z); 
                m_btee2marker.setOrigin(btTrans);
                m_ee2marker.setLocalPos(cVector3d(q_dual.x, q_dual.y, q_dual.z));
                // Change to btVector and Matrix
                m_flagHE = true;
            }

            m_markerPtr = m_worldPtr->getRigidBody("marker_body");
            if(!m_markerPtr){
                cerr << "WARNING! No Marker named marker_body found." << endl;
            }
        }

        if (node["pivot"]){
            cout << "Pivot Calibration" << endl;

            string toolTipName = node["pivot"]["tooltip name"].as<string>();
            m_toolTipPtr = m_worldPtr->getRigidBody(toolTipName);

            if(!m_toolTipPtr){
                cerr << "ERROR! NO Tooltip named: " << toolTipName << endl;
                return -1;
            }

            // Get marker in AMBF
            string markerName = node["pivot"]["marker name"].as<string>();
            m_toolPtr = m_worldPtr->getRigidBody(markerName);
            
            if(!m_toolPtr){
                cerr << "WARNING! No marker named " << markerName << "found." << endl;
            }

            if(node["pivot"]["resolution"]){
                m_pivotRes = node["pivot"]["resolution"].as<double>();
            }

            // Setting up CRTK communication
            string nspace = node["pivot"]["namespace"].as<string>();
            m_toolInterface = new CRTKInterface("/" + markerName + "/");

            if(!m_markerPtr){
                m_markerPtr = m_worldPtr->getRigidBody("marker_body");
                if(!m_markerPtr){
                    cerr << "WARNING! No Marker named marker_body found." << endl;
                }
            }

            if(node["pivot"]["number of points"]){
                m_numPivot = node["pivot"]["number of points"].as<int>();
            }

            if(node["pivot"]["registered pivot result"]){
                double x = node["pivot"]["registered pivot result"]["t_tip"]["x"].as<double>();
                double y = node["pivot"]["registered pivot result"]["t_tip"]["y"].as<double>();
                double z = node["pivot"]["registered pivot result"]["t_tip"]["z"].as<double>();

                m_marker2tip.set(x, y, z);
                m_flagPivot = true;
            }
            m_burrMesh = new cShapeSphere(0.002);
            m_burrMesh->setRadius(0.002);
            m_burrMesh->m_material->setBlack();
            m_burrMesh->m_material->setShininess(0);
            m_burrMesh->m_material->m_specular.set(0, 0, 0);
            m_burrMesh->setShowEnabled(true);

            m_worldPtr->addSceneObjectToWorld(m_burrMesh);

        }


        return 1;

}

void afRegistrationPlugin::reset(){
    cerr << "INFO! PLUGIN RESET CALLED" << endl;
}

bool afRegistrationPlugin::close(){
    delete m_registrationStatusLabel;
    delete m_savedPointsListLabel;
    return -1;
}

void saveDataToCSV(string fileName, vector<cTransform> vecTransform){
    std::ofstream file;
    time_t now = time(0);

    file.open("./data/" + fileName);
    for (int i=0; i < vecTransform.size(); i++){
        file << to_string(i) +  ", " +  vecTransform[i].getLocalPos().str(6) + ",";
        cQuaternion qRot;
        qRot.fromRotMat(vecTransform[i].getLocalRot());
        file << to_string(qRot.x) + ", " + to_string(qRot.y) + ", " + to_string(qRot.z) + ", " + to_string(qRot.w) + "\n";
    }
    file.close();
}

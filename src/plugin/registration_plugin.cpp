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
using namespace std;

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
        cout<< cmd_opts << endl;
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

    // Initiallize camera
    vector<string> cameraNames = {"main_camera", "cameraL", "cameraR", "stereoLR"};
    bool  initcam = initCamera(cameraNames);
    
    if (initcam && initLabels()){
        cerr << "SUCCESSFULLY Initialize Camera and Labels" << endl;
    }
    
    else{
        cerr << "ERROR! FAILED To Initialize Camera and/or Lables" << endl;
        return -1;
    }

    // Create burr mesh
    m_burrMesh = new cShapeSphere(0.001);
    m_burrMesh->setRadius(0.001);
    m_burrMesh->m_material->setRed();
    m_burrMesh->m_material->setShininess(0);
    m_burrMesh->m_material->m_specular.set(0, 0, 0);
    m_burrMesh->setLocalPos(cVector3d(0.0, 0.0, 0.0));
    m_burrMesh->setShowEnabled(false);  
    m_worldPtr->addSceneObjectToWorld(m_burrMesh);

    // When config file is defined
    if(!config_filepath.empty()){
        return readConfigFile(config_filepath);
    }

    // If the configuration file is not defined provide error
    else{
        cerr << "ERROR! NO configuration file specified." << endl;
        return -1;
    }
}

// Initialize labels and panel in the simulation
bool afRegistrationPlugin::initLabels(){
    // create a font
    cFontPtr font = NEW_CFONTCALIBRI20();

    // Active Object panel
    m_registrationStatusLabel = new cLabel(font);
    m_registrationStatusLabel->m_fontColor.setBlack();
    m_registrationStatusLabel->setCornerRadius(5, 5, 5, 5);
    m_registrationStatusLabel->setShowPanel(true);
    m_registrationStatusLabel->setColor(cColorf(1.0, 1.0, 1.0, 1.0));
    m_panelManager.addPanel(m_registrationStatusLabel, 0.7, 0.85, PanelReferenceOrigin::LOWER_LEFT, PanelReferenceType::NORMALIZED);
    m_panelManager.setVisible(m_registrationStatusLabel, true);

    // Objects List panel
    m_savedPointsListLabel = new cLabel(font);
    m_savedPointsListLabel->setFontScale(0.8);
    m_savedPointsListLabel->m_fontColor.setBlack();
    m_savedPointsListLabel->setCornerRadius(5, 5, 5, 5);
    m_savedPointsListLabel->setShowPanel(true);
    m_savedPointsListLabel->setColor(cColorf(1.0, 1.0, 1.0, 1.0));
    m_panelManager.addPanel(m_savedPointsListLabel, 0.7, 0.75, PanelReferenceOrigin::LOWER_LEFT, PanelReferenceType::NORMALIZED);
    m_panelManager.setVisible(m_savedPointsListLabel, true);

    return true;

}

// Look for the camera and store as a "main camera"
bool afRegistrationPlugin::initCamera(vector<string> cameraNames){
    cout << "> Initializing CAMERA ..." << endl;
    if (cameraNames.size() == 0){
        cerr << "ERROR! NO CAMERA Specified." << endl;
        return false;
    }
    
    // Loop around the cameras and look for the "main camera"
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

    // Set background
    cBackground* background = new cBackground();
    background->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(0.6f, 0.6f, 0.6f),
                                cColorf(0.6f, 0.6f, 0.6f));
    m_cameras["main_camera"]->getBackLayer()->addChild(background);

    return true;
}


// Define Key board shortcuts
void afRegistrationPlugin::keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods){
    if (a_mods == GLFW_MOD_CONTROL){
        if (a_key == GLFW_KEY_1){
            if (m_isHE){
                m_activeMode = RegistrationMode::HANDEYE;
                cerr << "Registration Mode changed to HANDEYE " << endl;
            }
            else{
                m_activeMode = RegistrationMode::UNREGISTERED;
            }
        }

        else if (a_key == GLFW_KEY_2){
            if(m_isPivot){
                m_activeMode = RegistrationMode::PIVOT;
                cerr << "Registration Mode changed to PIVOT " << endl;
            }
            else{
                m_activeMode = RegistrationMode::UNREGISTERED;
            }
        }
        
        else if (a_key == GLFW_KEY_3){
            if(m_isPointer){
                m_activeMode = RegistrationMode::POINTER;
                cerr << "Registration Mode changed to POINTER " << endl;
            }
            else{
                m_activeMode = RegistrationMode::UNREGISTERED;
            }
        }
    
        else if(a_key == GLFW_KEY_9){
            if (m_activeMode == RegistrationMode::POINTER || m_activeMode == RegistrationMode::PIVOT){
                m_savePoint = true;
                cerr << "Saving Tooltip location ..." << endl;
            }
        }
    }
}

void afRegistrationPlugin::addOptionDescription(string & text){
    if(m_isHE){
        text += "[Ctrl + 1]: Hand-Eye Registraion,";
    }
    else{
        text += "[Ctrl + 1]: None, ";
    }
    if(m_isPivot){
        text += "[Ctrl + 2]: Pivot Calibration, ";
    }
    else{
        text += "[Ctrl + 2]: None, ";
    }
    if(m_isPointer){
        text += "[Ctrl + 3]: Ponter Based Registration, ";
    }
    else{
        text += "[Ctrl + 3]: None, ";
    }
    if(m_isPointer){
        text += "[Ctrl + 9]: Sample Point";
    }
    else{
        text += "[Ctrl + 9]: None";
    }
}

// Graphics related updates
void afRegistrationPlugin::graphicsUpdate(){

    string m_savedLocationText = "--- List of Saved Locations ([Ctrl + 9] to sample Point) --- \n";
    string text;
    // For Registration Status Label
    switch (m_activeMode){
        case RegistrationMode::UNREGISTERED:
            text = "Registration Status: UNREGISTER \n";
            addOptionDescription(text);
            m_panelManager.setText(m_registrationStatusLabel, text);
            m_panelManager.setFontColor(m_registrationStatusLabel, cColorf(0.,0.0,0.0));
            break;
        
        case RegistrationMode::POINTER:
            m_panelManager.setText(m_registrationStatusLabel, "Registration Status: POINTER");
            m_panelManager.setFontColor(m_registrationStatusLabel, cColorf(0.,0.,0.));

            // Saving the saved points location as text and show on the screen.
            for (int i=0; i < m_savedPointMeshList.size(); i++){   
                cVector3d trans = m_savedPointMeshList[i]->getLocalPos();
                m_savedLocationText += "Point " + to_string(i) + ": " + trans.str(5);
                if(i < m_savedPointMeshList.size()-1){
                    m_savedLocationText += "\n";
                }
            }
            m_panelManager.setText(m_savedPointsListLabel, m_savedLocationText);

            for (size_t i = 0; i < m_visualPointsInModel.size(); i++){
                if (i == m_savedPointMeshList.size())
                    m_visualPointsInModel[i]->m_material->setRed();
                else{
                    m_visualPointsInModel[i]->m_material->setGreen();
                }
            }
            
            break;

        case RegistrationMode::PIVOT:
            m_panelManager.setText(m_registrationStatusLabel, "Registration Status: PIVOT");
            m_panelManager.setFontColor(m_registrationStatusLabel, cColorf(0.,0.,0.));
            
            if (m_manualPivot){
                // Saving the saved points location as text and show on the screen.
                for (int i=0; i < m_savedPivotPoints.size(); i++){   
                    cVector3d trans = m_savedPivotPoints[i].getLocalPos();
                    m_savedLocationText += "Point " + to_string(i) + ": " + trans.str(5);
                    if(i < m_savedPivotPoints.size()-1){
                        m_savedLocationText += "\n";
                    }
                }
            }
            
            m_panelManager.setText(m_savedPointsListLabel, m_savedLocationText);
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

void afRegistrationPlugin::applybtTransformToRigidBody(afRigidBodyPtr bodyPtr, btTransform& trans){
    // Move the object to the registered location
    btTransform currentTransform, Tcommand;

    bodyPtr->m_bulletRigidBody->getMotionState()->getWorldTransform(currentTransform);
    Tcommand.mult(currentTransform, trans); 

    bodyPtr->m_bulletRigidBody->getMotionState()->setWorldTransform(Tcommand);
    bodyPtr->m_bulletRigidBody->setWorldTransform(Tcommand);
}

// Physics related updates
void afRegistrationPlugin::physicsUpdate(double dt){
    if (m_activeMode == RegistrationMode::POINTER){
        
        // Generate and store the location when the keyboard shortcut is pressed
        if (m_savePoint){
            // Create red sphere when the saving the location
            cShapeSphere* savedPointMesh = new cShapeSphere(0.001);
            savedPointMesh->setRadius(0.001);
            savedPointMesh->m_material->setRed();
            savedPointMesh->m_material->setShininess(0);
            savedPointMesh->m_material->m_specular.set(0, 0, 0);
            savedPointMesh->setShowEnabled(true);

            // If there is a pointerToolTip rigid body specified
            if (m_pointerToolTipPtr){
                savedPointMesh->setLocalPos(m_pointerToolTipPtr->getLocalPos()); // Save tooltip location (CAD model)
            }

            // If the HE and pivot registration is being done and specified in the config file
            else if ((m_HEDone && m_pivotDone) || (m_HEDefined && m_pivotDefined)){
                savedPointMesh->setLocalPos(m_burrMesh->getLocalPos()); // Save burr mesh location/
                m_burrMesh->setShowEnabled(true);
            }

            // Add the saved point mesh to the world
            m_worldPtr->addSceneObjectToWorld(savedPointMesh);
            
            // Store the points
            m_savedPointMeshList.push_back(savedPointMesh);
            m_savePoint = false;
        }

        // TODO: Add/Remove saved Points if needed
        // Once all the points are saved
        if (m_savedPointMeshList.size() == m_pointsPtr.size()){
            cerr << "Successfully collected " << m_pointsPtr.size() << " points..." << endl;

            vector<cVector3d> pointsIn;
            vector<cVector3d> pointsOut;
            for (int idx=0; idx<m_savedPointMeshList.size(); idx++){
                pointsIn.push_back(m_pointsPtr[idx]->getLocalPos());
                pointsOut.push_back(m_savedPointMeshList[idx]->getLocalPos());
            }

            // Perform ICP registration
            // bool resultRegist = m_pointCloudRegistration.ICPRegistration(pointsIn, pointsOut, m_registeredTransform);

            // Perform Point set Registration
            vector<cVector3d> registeredPoints;
            bool resultRegist = m_pointCloudRegistration.PointSetRegistration(pointsIn, pointsOut, m_registeredTransform, registeredPoints);

            // If the result was not correct erase the collected points
            if (!resultRegist){
                m_savedPointMeshList.clear();
                cerr << "Point set registration failed. Try again." << endl;
            }

            // Visualize the registered points
            for (size_t i = 0; i < registeredPoints.size(); i++){
                cShapeSphere* point = new cShapeSphere(0.001);
                point->setRadius(0.001);
                point->m_material->setBlue();
                point->m_material->setShininess(0);
                point->m_material->m_specular.set(0, 0, 0);
                point->setShowEnabled(true);
                point->setLocalPos(registeredPoints[i]);
                m_worldPtr->addSceneObjectToWorld(point);
            }

            if (resultRegist){
                // Change mode to "REGISTERED"
                m_activeMode = RegistrationMode::REGISTERED;
                applybtTransformToRigidBody(m_registeringObject, m_registeredTransform);

                for (const auto visualSphere : m_visualPointsInModel){
                    visualSphere->setShowEnabled(false);
                }
            }
        }
    }

    else if (m_activeMode == RegistrationMode::HANDEYE){
        cTransform measured_cp = m_HEtoolInterface->measured_cp();
        m_registeredText = "WARNING! No tool location published \nCheck your tracker!!\n";

        if (measured_cp.getLocalPos().length() > 0.0){
            m_registeredText = "";
        }
        
        m_registeredText += "WARNING! No robot related topic published";
    
        // ToDo: Change this to Robot Mode
        if(measured_cp.getLocalPos().length() > 0.0 && !m_HEDone){
            // Erase the warning if there is measured_cf
            m_registeredText = "Saving Points...\n";

            // get trackerLocation data from rostopics
            cTransform collectedPoint = m_HEtoolInterface->measured_cp();
            collectedPoint.setLocalPos(collectedPoint.getLocalPos());

            cTransform collectedReference;
            if (m_HEreferenceInterface){
                collectedReference = m_HEreferenceInterface->measured_cp();
                collectedReference.invert();
            }
            else{
                collectedReference.identity();
            }

            if (m_savedTracker2Points.size() == 0){
                m_savedTracker2Points.push_back(collectedPoint);
                m_savedRef2Points.push_back(collectedReference * collectedPoint);
                m_savedAMBFPoints.push_back(m_HEeePtr->getLocalTransform());
                m_savedRobotPoints.push_back(m_HErobotInterface->measured_cp());
            }
            else {
                // Save only the new collected points which are far enough from old points
                if ((m_savedTracker2Points.back().getLocalPos() - collectedPoint.getLocalPos()).length() > m_trackRes){
                    m_savedTracker2Points.push_back(collectedPoint);
                    m_savedRef2Points.push_back(collectedReference * collectedPoint);
                    m_savedAMBFPoints.push_back(m_HEeePtr->getLocalTransform());
                    m_savedRobotPoints.push_back(m_HErobotInterface->measured_cp());
                }
            }

            // Once you collected enough points for the calibration
            if (m_savedTracker2Points.size() > m_numHE && !m_HEDone){
                m_registeredText = "Saving Points into csv file.";
                saveDataToCSV("HE_trackerTomarker.csv", m_savedTracker2Points);             
                saveDataToCSV("HE_referenceTomarker.csv", m_savedRef2Points);             
                saveDataToCSV("HE_ambfToEE.csv", m_savedAMBFPoints);   
                saveDataToCSV("HE_worldToEE.csv", m_savedRobotPoints);   
                m_registeredText = "[INFO] Saved to /data/ folder!";

                // TODO: Implement handeye here
                // m_handEyeCalibration.calibrate(m_savedRobotPoints, m_savedPoints, m_ee2marker, m_tracker);
            
                cerr << "Saved to /data/ folder!" << endl;
                m_HEDone = true;
            }

            else if(m_savedTracker2Points.size() > 0){
                m_registeredText += "Number of saved Points: " + to_string(m_savedTracker2Points.size()) + "/" + to_string(m_numHE);
            }
        }
    }
    
    else if (m_activeMode == RegistrationMode::PIVOT){

        cTransform measured_cp;
        m_registeredText = "WARNING! No tool location published \nCheck your tracker!!\n";

        // Perform Pivot calibration method using Optical Tracker
        if(!m_manualPivot){
            measured_cp = m_pivotToolInterface->measured_cp();
            if (measured_cp.getLocalPos().length() > 0.0 && !m_pivotDone){
                // Erase the warning if there is measured_cf
                m_registeredText = "Saving Points...\n";

                // get trackerLocation data from rostopics
                cTransform collectedPoint = m_pivotToolInterface->measured_cp();

                cTransform collectedReference;
                if (m_pivotReferenceInterface){
                    collectedReference = m_pivotReferenceInterface->measured_cp();
                    collectedReference.invert();
                }
                else{
                    collectedReference.identity();
                }

                if (m_savedPivotPoints.size() == 0){
                    m_savedPivotPoints.push_back(collectedPoint);
                    m_savedPivotRef2Points.push_back(collectedReference * collectedPoint);
                }
                else {
                    // Save only the new collected points which are far enough from old points
                    if ((m_savedPivotPoints.back().getLocalPos() - collectedPoint.getLocalPos()).length() > m_pivotRes){
                        // Add mesh to the scene
                        cShapeSphere* savedPivotPointMesh = new cShapeSphere(0.001);
                        savedPivotPointMesh->setRadius(0.001);
                        savedPivotPointMesh->m_material->setBlue();
                        savedPivotPointMesh->m_material->setShininess(0);
                        savedPivotPointMesh->m_material->m_specular.set(0, 0, 0);
                        savedPivotPointMesh->setShowEnabled(true);
                        savedPivotPointMesh->setLocalPos(collectedPoint.getLocalPos());
                        m_worldPtr->addSceneObjectToWorld(savedPivotPointMesh);

                        m_savedPivotPoints.push_back(collectedPoint);
                        m_savedPivotRef2Points.push_back(collectedReference * collectedPoint);
                    }
                }

                // Once you collected enough points for the calibration
                if (m_savedPivotPoints.size() > m_numPivot){
                    cVector3d dimple;
                    cVector3d test;
                    m_pivotCalibration.calibrate(m_savedPivotPoints, m_marker2tip, dimple);

                    if (m_pivotReferenceInterface){
                        cout << "Pivot calibration result w.r.t reference." << endl;
                        m_pivotCalibration.calibrate(m_savedPivotRef2Points, test, dimple);
                    }

                    // If you want to save the points
                    m_registeredText = "Saving Points into csv file.";
                    saveDataToCSV("Pivot_trackerTomarker.csv", m_savedPivotPoints);             
                    saveDataToCSV("Pivot_referenceTomarker.csv", m_savedPivotRef2Points);             
                    m_registeredText = "[INFO] Saved to /data/ folder!";
                    cerr << "Saved to /data/ folder!" << endl;
                    
                    m_pivotDone = true;
                }

                else if(m_savedPivotPoints.size() > 0){
                    m_registeredText += "Number of saved Points: " + to_string(m_savedPivotPoints.size()) + "/" + to_string(m_numPivot);
                }
            }
        }

        // Perform pivot calibration without optical tracker. Robot base pivot calibration
        if(m_manualPivot){
            cTransform measured_cp = m_pivotToolInterface->measured_cp();

            if (measured_cp.getLocalPos().length() > 0.0 && !m_pivotDone){
                // Erase the warning if there is measured_cf
                m_registeredText = "Saving Points...\n";

                if (m_savePoint){
                    if (m_savedPivotPoints.size() == 0){
                    m_savedPivotPoints.push_back(measured_cp);
                    }
                    else {
                        // Save only the new collected points which are far enough from old points
                        if ((m_savedPivotPoints.back().getLocalPos() - measured_cp.getLocalPos()).length() > m_pivotRes){
                            m_savedPivotPoints.push_back(measured_cp);
                        }
                    }
                    m_savePoint = false;
                }
            }

            // Once you collected enough points for the calibration
            if (m_savedPivotPoints.size() > m_numPivot){
                cVector3d dimple;
                cVector3d test;
                m_pivotCalibration.calibrate(m_savedPivotPoints, m_ee2tip, dimple);

                // If you want to save the points
                m_registeredText = "Saving Points into csv file.";
                saveDataToCSV("Pivot_robotTomarker.csv", m_savedPivotPoints);             
                m_registeredText = "[INFO] Saved to /data/ folder!";
                cerr << "Saved to /data/ folder!" << endl;
                
                m_pivotDone = true;
                m_activeMode = RegistrationMode::REGISTERED;
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
    if (m_HEDone || m_HEDefined){
        if(m_HEeePtr && m_HEmarkerPtr){
            // Use btTransform to move the Marker
            btTransform Tcommand;
            m_HEeePtr->m_bulletRigidBody->getMotionState()->getWorldTransform(Tcommand);
            Tcommand.mult(Tcommand, m_btee2marker);
            m_HEmarkerPtr->m_bulletRigidBody->getMotionState()->setWorldTransform(Tcommand);
            m_HEmarkerPtr->m_bulletRigidBody->setWorldTransform(Tcommand);
        }
    }

    // If pivot calibration is done
    if (m_pivotDone || m_pivotDefined){
        if (m_pivotMarkerPtr){
            cVector3d tmp;
            m_ee2marker.mulr(m_marker2tip, tmp);
            cVector3d tip;
            m_pivotMarkerPtr->getLocalTransform().mulr(m_ee2tip, tip);
            m_burrMesh->setLocalPos(tip);
            m_burrMesh->setShowEnabled(true);
        }
    }
    
    // If both Handeye calibration and pivot calibration are done
    if(m_HEDefined && m_pivotDefined){
        cVector3d finalTransform;
        m_ee2marker.mulr(m_marker2tip, finalTransform);
        m_registeredText += "EE2Tooltip: " + finalTransform.str(6);
    }
}

int afRegistrationPlugin::readConfigFile(string config_filepath){
        cerr << "> loading the user defined configuration file..." << endl; 
        cerr << config_filepath << endl;

        //Load the user defined object here. 
        YAML::Node node = YAML::LoadFile(config_filepath);
        
        // Check whether pointer based registration is needed or not
        if (node["pointer"]){
            cout << "> Pointer based Registration" << endl;
            m_isPointer = true;
            // Get the name of the tooltip object
            string toolTipName = node["pointer"]["tooltip name"].as<string>();

            m_pointerToolTipPtr = m_worldPtr->getRigidBody(toolTipName);

            if(!m_pointerToolTipPtr){
                cerr << "ERROR! NO Tooltip named: " << toolTipName << endl;
                return -1;
            }

            int numPoints = node["pointer"]["name of points"].size();
            // Load all the points
            for (int i=0; i < numPoints; i++){

                afRigidBodyPtr objectPtr;
                objectPtr = m_worldPtr->getRigidBody(node["pointer"]["name of points"][i].as<string>());
                
                
                if(objectPtr){
                    m_pointsPtr.push_back(objectPtr);

                    // Create visual green sphere
                    cShapeSphere* visualMesh = new cShapeSphere(0.001);
                    visualMesh->setRadius(0.001);
                    visualMesh->m_material->setColorf(0,1,0,1);
                    visualMesh->m_material->setShininess(0);
                    visualMesh->m_material->m_specular.set(0, 0, 0);
                    visualMesh->setLocalPos(objectPtr->getLocalPos());
                    visualMesh->setShowEnabled(true);  
                    m_worldPtr->addSceneObjectToWorld(visualMesh);

                    // Store visualPoints in the vector
                    m_visualPointsInModel.push_back(visualMesh);
                }
                else{
                    cerr << "WARNING! No point named " << node["pointer"]["name of points"][i].as<string>() << " found." << endl;
                }

            }

            cerr << numPoints << " points are specified as Keypoints" << endl;
            
            m_registeringObject = m_worldPtr->getRigidBody(node["pointer"]["object name"].as<string>());

            if(!m_registeringObject){
                cerr << "ERROR! No object named " << node["pointer"]["object name"].as<string>() << " found." << endl;
                
                return -1;
            }
            else{
                // Get pointer to camera
                afCameraPtr model_camera = m_worldPtr->getCamera("model_camera");

                if (model_camera){
                    // Set background
                    cBackground* background = new cBackground();
                    background->setCornerColors(cColorf(0.2f, 0.2f, 0.2f),
                                                cColorf(0.2f, 0.2f, 0.2f),
                                                cColorf(0.2f, 0.2f, 0.2f),
                                                cColorf(0.2f, 0.2f, 0.2f));
                    model_camera->getBackLayer()->addChild(background);

                    // Load Camera related parameters
                    cVector3d camLocation;
                    cVector3d camLookAt;
                    cVector3d camUp;
                    if (node["pointer"]["camera"].IsDefined()){
                        YAML::Node camLocationNode = node["pointer"]["camera"]["location_offset"];
                        camLocation = m_registeringObject->getLocalPos() + to_cVector3d(adf_loader_1_0::ADFUtils::positionFromNode(&camLocationNode));
                        YAML::Node camLookAtNode = node["pointer"]["camera"]["look at"];
                        camLookAt = to_cVector3d(adf_loader_1_0::ADFUtils::positionFromNode(&camLookAtNode));
                        YAML::Node camUpNode = node["pointer"]["camera"]["up"];
                        camUp = to_cVector3d(adf_loader_1_0::ADFUtils::positionFromNode(&camUpNode));
                    }
                    else{
                        camLocation = m_registeringObject->getLocalPos() + 0.5 * cVector3d(0, 1.0, 0.0);
                        camLookAt = cVector3d(0, -1.0, 0.0);
                        camUp = cVector3d(0, 0.0, 1.0);
                    }

                    // Set camera related parameters
                    model_camera->setView(camLocation, camLookAt, camUp);

                    // Set light at the same location
                    afLightPtr model_light = m_worldPtr->getLight("model_light");
                    model_light->setLocalPos(model_camera->getLocalPos());
                    model_light->setDir(camLookAt);
                }
            }
        }

        // Check whether hand-eye calibration is needed or not
        if (node["hand eye"]){
            cout << "> Hand Eye Calibration" << endl;
            m_isHE = true;

            // Get marker in AMBF
            string markerName = node["hand eye"]["marker name"].as<string>();
            m_HEmarkerPtr = m_worldPtr->getRigidBody(markerName);
            
            if(!m_HEmarkerPtr){
                cerr << "WARNING! No marker named " << markerName << " found." << endl;
            }

            // Setting up CRTK communication
            string robot_nspace = node["hand eye"]["robot namespace"].as<string>();
            string marker_nspace = node["hand eye"]["marker namespace"].as<string>();
            m_HErobotInterface = new CRTKInterface(robot_nspace);
            m_HEtoolInterface = new CRTKInterface(marker_nspace);

            if (node["hand eye"]["reference namespace"]){
                m_HEreferenceInterface = new  CRTKInterface(node["hand eye"]["reference namespace"].as<string>());
            }

            // Get EE rigid body in AMBF
            string EEName = node["hand eye"]["EE name"].as<string>();
            m_HEeePtr = m_worldPtr->getRigidBody(EEName);
            
            if(!m_HEeePtr){
                cerr << "ERROR! No Endeffector pose named " << EEName << " found." << endl;
            }

            if(node["hand eye"]["optical tracker name"]){
                string trackerName = node["hand eye"]["optical tracker name"].as<string>();
                m_HEtrackerPtr = m_worldPtr->getRigidBody(trackerName);

                if(!m_HEtrackerPtr){
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
                m_HEDefined = true;
            }
        }

        if (node["pivot"]){
            cout << "> Pivot Calibration" << endl;
            m_isPivot = true;

            // Setting up CRTK communication
            string marker_nspace = node["pivot"]["marker namespace"].as<string>();
            m_pivotToolInterface = new CRTKInterface(marker_nspace);

            if (node["pivot"]["reference namespace"]){
                m_pivotReferenceInterface = new CRTKInterface(node["pivot"]["reference namespace"].as<string>());
            }

            // Get marker in AMBF
            string markerName = node["pivot"]["marker name"].as<string>();
            m_pivotMarkerPtr = m_worldPtr->getRigidBody(markerName);
            
            if(!m_pivotMarkerPtr){
                cerr << "WARNING! No marker named " << markerName << "found." << endl;
            }
            
            if (node["pivot"]["type"]){
                if (node["pivot"]["type"].as<string>() == "AUTO"){
                    m_manualPivot = false;
                }
                else if (node["pivot"]["type"].as<string>() == "MANUAL"){
                    m_manualPivot = true;
                }
                else{
                    cerr << "ERROR! Type has to be either 'AUTO' or 'MANUAL'" << endl;
                    return -1;
                }
            }

            if(node["pivot"]["resolution"]){
                m_pivotRes = node["pivot"]["resolution"].as<double>();
            }

            if(node["pivot"]["number of points"]){
                m_numPivot = node["pivot"]["number of points"].as<int>();
            }

            if(node["pivot"]["registered pivot result"]){
                double x = node["pivot"]["registered pivot result"]["t_tip"]["x"].as<double>();
                double y = node["pivot"]["registered pivot result"]["t_tip"]["y"].as<double>();
                double z = node["pivot"]["registered pivot result"]["t_tip"]["z"].as<double>();

                m_marker2tip.set(x, y, z);
                m_pivotDefined = true;
            }
        }
        cerr << "SUCCESSFULLY Initialized plugin!!" << endl;

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

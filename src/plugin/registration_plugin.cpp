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
            ("registration_config", p_opt::value<string>()->default_value(""), "Name of specfile for spacenav control")
            ("tf_list", p_opt::value<string>()->default_value(""), "ambf_tf_plugin config. Used to read the tool tip for axis calibration");

    p_opt::variables_map var_map;
    p_opt::store(p_opt::command_line_parser(argc, argv).options(cmd_opts).allow_unregistered().run(), var_map);
    p_opt::notify(var_map);

    if(var_map.count("info")){
        cout<< cmd_opts << endl;
        return -1;
    }

    // Loading options 
    string config_filepath = var_map["registration_config"].as<string>();
    m_tfConfigFilepath = var_map["tf_list"].as<string>();

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
    m_panelManager.addPanel(m_registrationStatusLabel, 0.01, 0.85, PanelReferenceOrigin::LOWER_LEFT, PanelReferenceType::NORMALIZED);
    m_panelManager.setVisible(m_registrationStatusLabel, true);

    // Objects List panel
    m_savedPointsListLabel = new cLabel(font);
    m_savedPointsListLabel->setFontScale(0.8);
    m_savedPointsListLabel->m_fontColor.setBlack();
    m_savedPointsListLabel->setCornerRadius(5, 5, 5, 5);
    m_savedPointsListLabel->setShowPanel(true);
    m_savedPointsListLabel->setColor(cColorf(1.0, 1.0, 1.0, 1.0));
    m_panelManager.addPanel(m_savedPointsListLabel, 0.01, 0.75, PanelReferenceOrigin::LOWER_LEFT, PanelReferenceType::NORMALIZED);
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
                setActiveMode(RegistrationMode::HANDEYE);
                cerr << "Registration Mode changed to HANDEYE " << endl;
            }
            else{
                setActiveMode(RegistrationMode::UNREGISTERED);
            }
        }

        else if (a_key == GLFW_KEY_2){
            if(m_isPivot){
                setActiveMode(RegistrationMode::PIVOT);
                cerr << "Registration Mode changed to PIVOT " << endl;
            }
            else{
                setActiveMode(RegistrationMode::UNREGISTERED);
            }
        }
        
        else if (a_key == GLFW_KEY_3){
            if(m_isPointer){
                setActiveMode(RegistrationMode::POINTER);
                cerr << "Registration Mode changed to POINTER " << endl;
            }
            else{
                setActiveMode(RegistrationMode::UNREGISTERED);
            }
        }

        else if (a_key == GLFW_KEY_4){
            if(m_isAxis){
                setActiveMode(RegistrationMode::AXIS);
                cerr << "Registration Mode changed to AXIS " << endl;
            }
            else{
                cerr << "WARNING! Axis calibration is not configured. Add an 'axis' or 'pivot' section to the registration config." << endl;
                setActiveMode(RegistrationMode::UNREGISTERED);
            }
        }
    
        else if(a_key == GLFW_KEY_9){
            if (m_activeMode == RegistrationMode::POINTER || m_activeMode == RegistrationMode::PIVOT){
                m_savePoint = true;
                cerr << "Saving Tooltip location ..." << endl;
            }
        }

        // Restart the active calibration (handled in physicsUpdate)
        else if (a_key == GLFW_KEY_D && a_action == GLFW_PRESS){
            if (m_activeMode == RegistrationMode::PIVOT || m_activeMode == RegistrationMode::AXIS){
                m_restartRequested = true;
            }
        }

        else if (a_key == GLFW_KEY_W && a_action == GLFW_PRESS){
            if (m_activeMode == RegistrationMode::REGISTERED){
                cerr << "[INFO!!] Rewriting ADF file ..." << endl;
                // Quary for rewriting the ADF file
                reWriteADFfile(m_registeringObjectADFFilePath, m_registeredTransform);
            }
            else if ((m_activeMode == RegistrationMode::PIVOT && m_pivotDone) ||
                     (m_activeMode == RegistrationMode::AXIS && m_axisDone)){
                saveCalibrationToTF();
            }
        }
    }
}

// Change the registration mode and only show the samples of the active calibration
void afRegistrationPlugin::setActiveMode(RegistrationMode mode){
    m_activeMode = mode;
    m_tfSaveText.clear();
    for (auto mesh: m_pivotPointMeshes){
        mesh->setShowEnabled(mode == RegistrationMode::PIVOT);
    }
    for (auto mesh: m_axisPointMeshes){
        mesh->setShowEnabled(mode == RegistrationMode::AXIS);
    }
}

void afRegistrationPlugin::addOptionDescription(string & text){
    auto option = [&](const string& key, const string& name, RegistrationMode mode, bool configured, bool done){
        if (!configured){
            return;
        }
        text += (m_activeMode == mode) ? "> " : "   ";
        text += "[Ctrl + " + key + "]: " + name;
        if (done){
            text += " (done)";
        }
        text += "\n";
    };
    option("1", "Hand-Eye Registration", RegistrationMode::HANDEYE, m_isHE, m_HEDone);
    option("2", "Pivot Calibration", RegistrationMode::PIVOT, m_isPivot, m_pivotDone);
    option("3", "Pointer Based Registration", RegistrationMode::POINTER, m_isPointer, m_activeMode == RegistrationMode::REGISTERED);
    option("4", "Axis Calibration", RegistrationMode::AXIS, m_isAxis, m_axisDone);
    if (m_isPointer || m_manualPivot){
        text += "   [Ctrl + 9]: Sample Point\n";
    }
    if (m_isPivot || m_isAxis){
        text += "   [Ctrl + D]: Restart active calibration\n";
        text += "   [Ctrl + W]: Save result of the active calibration";
    }
}

// Stack the panels from the upper left corner of the main camera so they are fully visible
void afRegistrationPlugin::layoutPanels(){
    afCameraPtr cam = m_cameras["main_camera"];
    if (!cam || cam->m_width <= 0 || cam->m_height <= 0){
        return;
    }
    double margin = 10.0;
    double xpos = margin / cam->m_width;
    double top = cam->m_height - margin;

    top -= m_registrationStatusLabel->getHeight();
    m_panelManager.setPos(m_registrationStatusLabel, xpos, top / cam->m_height, PanelReferenceOrigin::LOWER_LEFT, PanelReferenceType::NORMALIZED);

    top -= margin + m_savedPointsListLabel->getHeight();
    m_panelManager.setPos(m_savedPointsListLabel, xpos, top / cam->m_height, PanelReferenceOrigin::LOWER_LEFT, PanelReferenceType::NORMALIZED);
}

// Graphics related updates
void afRegistrationPlugin::graphicsUpdate(){

    string m_savedLocationText = "--- List of Saved Locations ([Ctrl + 9] to sample Point) --- \n";
    string status;
    string info;
    cColorf statusColor(0., 0., 0.);
    // For Registration Status Label
    switch (m_activeMode){
        case RegistrationMode::UNREGISTERED:
            status = "UNREGISTERED";
            break;
        
        case RegistrationMode::POINTER:
            status = "POINTER";

            // Saving the saved points location as text and show on the screen.
            for (int i=0; i < m_savedPointMeshList.size(); i++){   
                cVector3d trans = m_savedPointMeshList[i]->getLocalPos();
                m_savedLocationText += "Point " + to_string(i) + ": " + trans.str(5);
                if(i < m_savedPointMeshList.size()-1){
                    m_savedLocationText += "\n";
                }
            }
            info = m_savedLocationText;

            for (size_t i = 0; i < m_visualPointsInModel.size(); i++){
                if (i == m_savedPointMeshList.size()){
                    m_visualPointsInModel[i]->m_material->setRed();
                    m_visualPointsInModel[i]->m_material->setShininess(0);
                    m_visualPointsInModel[i]->m_material->m_specular.set(0, 0, 0);
                }
                else{
                    m_visualPointsInModel[i]->m_material->setGreen();
                    m_visualPointsInModel[i]->m_material->setShininess(0);
                    m_visualPointsInModel[i]->m_material->m_specular.set(0, 0, 0);
                }
            }
            
            break;

        case RegistrationMode::PIVOT:
            status = "PIVOT";
            info = m_pivotText;
            
            if (m_manualPivot && !m_pivotDone){
                // Saving the saved points location as text and show on the screen.
                info += "\n" + m_savedLocationText;
                for (int i=0; i < m_savedPivotPoints.size(); i++){   
                    cVector3d trans = m_savedPivotPoints[i].getLocalPos();
                    info += "Point " + to_string(i) + ": " + trans.str(5);
                    if(i < m_savedPivotPoints.size()-1){
                        info += "\n";
                    }
                }
            }
            break;

        case RegistrationMode::AXIS:
            status = "AXIS";
            info = m_axisText;
            break;

        case RegistrationMode::HANDEYE:
            status = "HANDEYE";
            statusColor = cColorf(1., 0., 0.);
            info = m_registeredText;
            break;

        case RegistrationMode::REGISTERED:
            status = "REGISTERED";
            statusColor = cColorf(0., 0.5, 0.5);
            info = m_registeredText;
            break;
    }

    if (!m_tfSaveText.empty()){
        info += "\n" + m_tfSaveText;
    }

    string text = "Registration Status: " + status + "\n";
    addOptionDescription(text);
    m_panelManager.setText(m_registrationStatusLabel, text);
    m_panelManager.setFontColor(m_registrationStatusLabel, statusColor);
    m_panelManager.setText(m_savedPointsListLabel, info);
    m_panelManager.setFontColor(m_savedPointsListLabel, cColorf(0., 0., 0.));
    m_panelManager.setVisible(m_savedPointsListLabel, !info.empty());

    layoutPanels();
    m_panelManager.update();
}

void afRegistrationPlugin::applybtTransformToRigidBody(afRigidBodyPtr bodyPtr, btTransform& trans){
    // Move the object to the registered location
    btTransform currentTransform, Tcommand;

    bodyPtr->m_bulletRigidBody->getMotionState()->getWorldTransform(currentTransform);
    Tcommand = trans * currentTransform; 

    bodyPtr->m_bulletRigidBody->getMotionState()->setWorldTransform(Tcommand);
    bodyPtr->m_bulletRigidBody->setWorldTransform(Tcommand);
}

// Physics related updates
void afRegistrationPlugin::physicsUpdate(double dt){
    if (m_restartRequested){
        restartCalibration();
        m_restartRequested = false;
    }

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
        if (m_savedPointMeshList.size() == m_fiducialPointsPtr.size()){
            cerr << "Successfully collected " << m_fiducialPointsPtr.size() << " points..." << endl;

            vector<cVector3d> pointsIn;
            vector<cVector3d> pointsOut;
            for (int idx=0; idx<m_savedPointMeshList.size(); idx++){
                pointsIn.push_back(m_fiducialPointsPtr[idx]->getLocalPos());
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

            if (resultRegist){
                // Change mode to "REGISTERED"
                m_activeMode = RegistrationMode::REGISTERED;
                applybtTransformToRigidBody(m_registeringObject, m_registeredTransform);

                // Move model_camera once to the registered volume position.
                // Read from Bullet directly — getLocalPos() hasn't synced yet this step.
                if (m_modelCamera && m_registeringObject){
                    btVector3 btPos = m_registeringObject->m_bulletRigidBody->getWorldTransform().getOrigin();
                    cVector3d volumePos(btPos.x(), btPos.y(), btPos.z());
                    cVector3d camPos = volumePos + m_camLocationOffset;
                    cVector3d lookAt = volumePos + m_camLookAtOffset;
                    m_modelCamera->setView(camPos, lookAt, m_camUp);
                    if (m_modelLight){
                        m_modelLight->setLocalPos(camPos);
                        m_modelLight->setDir(lookAt - camPos);
                    }
                }

                for (const auto visualSphere : m_visualPointsInModel){
                    visualSphere->setShowEnabled(false);
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
            }
        }
    }

    else if (m_activeMode == RegistrationMode::HANDEYE){
        m_HErobotInterface->spin();
        m_HEreferenceInterface->spin();
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
        m_pivotToolInterface->spin();
        if (m_pivotReferenceInterface){
            m_pivotReferenceInterface->spin();
        }
        cTransform measured_cp;
        if (!m_pivotDone){
            m_pivotText = "WARNING! No tool location published \nCheck your tracker!!\n";
        }

        // Perform Pivot calibration method using Optical Tracker
        if(!m_manualPivot){
            measured_cp = m_pivotToolInterface->measured_cp();
            if (measured_cp.getLocalPos().length() > 0.0 && !m_pivotDone){
                // Erase the warning if there is measured_cf
                m_pivotText = "Saving Points...\n";

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

                // Save only the new collected points which are far enough from old points
                if (m_savedPivotPoints.size() == 0 ||
                    (m_savedPivotPoints.back().getLocalPos() - collectedPoint.getLocalPos()).length() > m_pivotRes){
                    // Add mesh to the scene
                    cShapeSphere* savedPivotPointMesh = new cShapeSphere(0.001);
                    savedPivotPointMesh->setRadius(0.001);
                    savedPivotPointMesh->m_material->setBlue();
                    savedPivotPointMesh->m_material->setShininess(0);
                    savedPivotPointMesh->m_material->m_specular.set(0, 0, 0);
                    savedPivotPointMesh->setShowEnabled(true);
                    savedPivotPointMesh->setLocalPos(collectedPoint.getLocalPos());
                    m_worldPtr->addSceneObjectToWorld(savedPivotPointMesh);
                    m_pivotPointMeshes.push_back(savedPivotPointMesh);

                    m_savedPivotPoints.push_back(collectedPoint);
                    m_savedPivotRef2Points.push_back(collectedReference * collectedPoint);
                }

                // Once you collected enough points for the calibration
                if (m_savedPivotPoints.size() > m_numPivot){
                    cVector3d dimple;
                    cVector3d test;
                    m_pivotCalibration.calibrate(m_savedPivotPoints, m_marker2tip, dimple, &m_pivotMeanError);

                    if (m_pivotReferenceInterface){
                        cout << "Pivot calibration result w.r.t reference." << endl;
                        m_pivotCalibration.calibrate(m_savedPivotRef2Points, test, dimple);
                    }

                    saveDataToCSV("Pivot_trackerTomarker.csv", m_savedPivotPoints);             
                    saveDataToCSV("Pivot_referenceTomarker.csv", m_savedPivotRef2Points);             
                    cerr << "Saved to /data/ folder!" << endl;

                    char buf[512];
                    snprintf(buf, sizeof(buf),
                             "Pivot calibration result:\n"
                             "position: {x: %.7g, y: %.7g, z: %.7g}\n"
                             "Mean calibration error: %.6g m\n",
                             m_marker2tip.x(), m_marker2tip.y(), m_marker2tip.z(), m_pivotMeanError);
                    m_pivotText = buf;
                    m_pivotText += savePromptText();
                    cerr << savePromptText() << endl;
                    
                    m_pivotDone = true;
                }

                else if(m_savedPivotPoints.size() > 0){
                    m_pivotText += "Number of saved Points: " + to_string(m_savedPivotPoints.size()) + "/" + to_string(m_numPivot);
                }
            }
        }

        // Perform pivot calibration without optical tracker. Robot base pivot calibration
        if(m_manualPivot){
            cTransform measured_cp = m_pivotToolInterface->measured_cp();

            if (measured_cp.getLocalPos().length() > 0.0 && !m_pivotDone){
                // Erase the warning if there is measured_cf
                m_pivotText = "Saving Points...\n";

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

                saveDataToCSV("Pivot_robotTomarker.csv", m_savedPivotPoints);             
                m_pivotText = "[INFO] Saved to /data/ folder!";
                cerr << "Saved to /data/ folder!" << endl;
                
                m_pivotDone = true;
                m_activeMode = RegistrationMode::REGISTERED;
            }

            else if(m_savedPivotPoints.size() > 0){
                m_pivotText += "Number of saved Points: " + to_string(m_savedPivotPoints.size()) + "/" + to_string(m_numPivot);
            }
        }
    }
    
    else if (m_activeMode == RegistrationMode::AXIS){
        m_axisToolInterface->spin();
        if (m_axisReferenceInterface){
            m_axisReferenceInterface->spin();
        }
        cTransform measured_cp = m_axisToolInterface->measured_cp();
        bool isTracked = measured_cp.getLocalPos().length() > 0.0;

        if (!m_axisDone){
            m_axisText = "Keep the tip on the pivot and rotate the tool about its shaft only.\n";
            if (!isTracked){
                m_axisText += "WARNING! No tool location published \nCheck your tracker!!\n";
            }
            else{
                cTransform collectedPoint = measured_cp;
                if (m_axisReferenceInterface){
                    cTransform collectedReference = m_axisReferenceInterface->measured_cp();
                    collectedReference.invert();
                    collectedPoint = collectedReference * measured_cp;
                }

                // Save only the new collected points which are far enough from old points
                if (m_savedAxisPoints.size() == 0 ||
                    (m_savedAxisPoints.back().getLocalPos() - collectedPoint.getLocalPos()).length() > m_axisRes){
                    cShapeSphere* savedAxisPointMesh = new cShapeSphere(0.001);
                    savedAxisPointMesh->m_material->setYellow();
                    savedAxisPointMesh->m_material->setShininess(0);
                    savedAxisPointMesh->m_material->m_specular.set(0, 0, 0);
                    savedAxisPointMesh->setShowEnabled(true);
                    savedAxisPointMesh->setLocalPos(measured_cp.getLocalPos());
                    m_worldPtr->addSceneObjectToWorld(savedAxisPointMesh);
                    m_axisPointMeshes.push_back(savedAxisPointMesh);

                    m_savedAxisPoints.push_back(collectedPoint);
                }
                m_axisText += "Number of saved Points: " + to_string(m_savedAxisPoints.size()) + "/" + to_string(m_numAxis);
            }

            // Once you collected enough points for the calibration
            if ((int)m_savedAxisPoints.size() >= m_numAxis){
                // Tool tip used to orient the axis:
                // pivot calibration of this session > axis section / tf config > pivot result in the config
                const cVector3d* marker2tip = nullptr;
                if (m_pivotDone && !m_manualPivot){
                    marker2tip = &m_marker2tip;
                    cerr << "Axis calibration uses the pivot calibration result of this session." << endl;
                }
                else if (m_axisTipDefined){
                    marker2tip = &m_axisMarker2tip;
                }
                else if (m_pivotDefined && !m_manualPivot){
                    marker2tip = &m_marker2tip;
                }

                saveDataToCSV("Axis_trackerTomarker.csv", m_savedAxisPoints);
                int res = m_axisCalibration.calibrate(m_savedAxisPoints, marker2tip, m_axisMeshAxis, m_axisRollOffset, m_axisResult);

                if (res < 0){
                    m_savedAxisPoints.clear();
                    cerr << "Axis calibration failed. Try again." << endl;
                }
                else{
                    m_axisDone = true;
                    m_axisLineTip = marker2tip ? *marker2tip : cVector3d(0, 0, 0);
                    char buf[1024];
                    snprintf(buf, sizeof(buf),
                             "Axis calibration result:\n"
                             "orientation: {r: %.6f, p: %.6f, y: %.6f}\n"
                             "Axis in marker frame: (%.4f, %.4f, %.4f)\n"
                             "Radius: %.4f m, plane RMS: %.5f m, radius RMS: %.5f m\n"
                             "Coverage: %.1f deg, axis spread: %.2f deg\n",
                             m_axisResult.roll, m_axisResult.pitch, m_axisResult.yaw,
                             m_axisResult.axisInMarker.x(), m_axisResult.axisInMarker.y(), m_axisResult.axisInMarker.z(),
                             m_axisResult.circleRadius, m_axisResult.planeRMS, m_axisResult.radiusRMS,
                             m_axisResult.angularCoverage * 180.0 / M_PI, m_axisResult.axisSpread * 180.0 / M_PI);
                    m_axisText = buf;
                    if (m_axisResult.signFromTip){
                        m_axisText += "Pivot point to axis distance: " + to_string(m_axisResult.tipToAxisDistance) + " m\n";
                    }
                    else{
                        m_axisText += "WARNING! No t_tip available: the axis sign is ambiguous.\n";
                    }
                    if (m_axisResult.angularCoverage < M_PI / 3.0){
                        m_axisText += "WARNING! Small angular coverage, rotate further for a better fit.\n";
                    }
                    m_axisText += savePromptText();
                    cerr << savePromptText() << endl;
                }
            }
        }
    }

    else if (m_activeMode == RegistrationMode::REGISTERED){
        // Saving text for the status monitor
        if (!m_isADFsaved)
        m_registeredText = "Registeration Result: \n Avg: " + to_string(m_registeredTransform.getOrigin().x()) + "," +
        to_string(m_registeredTransform.getOrigin().y()) + "," + to_string(m_registeredTransform.getOrigin().z()) + "\n" +
        "Press 'CTRL + W' to save the registered location in ADF file.";
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
            cVector3d tip;
            m_pivotMarkerPtr->getLocalTransform().mulr(m_manualPivot ? m_ee2tip : m_marker2tip, tip);
            m_burrMesh->setLocalPos(tip);
            m_burrMesh->setShowEnabled(true);
        }
    }

    // Show the calibrated axis attached to the marker, from the tip towards the back of the tool
    if (m_axisDone && m_pivotMarkerPtr){
        cTransform markerTrans = m_pivotMarkerPtr->getLocalTransform();
        cVector3d tip, back;
        markerTrans.mulr(m_axisLineTip, tip);
        markerTrans.mulr(m_axisLineTip + 0.2 * m_axisResult.axisInMarker, back);
        if (!m_axisLine){
            m_axisLine = new cShapeLine(tip, back);
            m_axisLine->setLineWidth(3.0);
            m_axisLine->m_colorPointA.setGreen();
            m_axisLine->m_colorPointB.setGreen();
            m_worldPtr->addSceneObjectToWorld(m_axisLine);
        }
        m_axisLine->m_pointA = tip;
        m_axisLine->m_pointB = back;
        m_axisLine->setShowEnabled(m_activeMode == RegistrationMode::AXIS);
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

            // Load registering object (ex. anatomical_origin)
            m_registeringObject = m_worldPtr->getRigidBody(node["pointer"]["object name"].as<string>());

            if (!m_registeringObject){
                cerr << "[ERROR!!] No object name " << node["pointer"]["object name"].as<string>() << endl;
            }
            
            // Load all the points
            for (int i=0; i < numPoints; i++){

                afRigidBodyPtr objectPtr;
                objectPtr = m_worldPtr->getRigidBody(node["pointer"]["name of points"][i].as<string>());

                // Apply anatomical origin transformation
                m_anatomicalOriginTransform = m_registeringObject->getLocalTransform();
                cTransform fiducialPose = objectPtr->getLocalTransform();

                objectPtr->setLocalTransform(m_anatomicalOriginTransform * fiducialPose);

                if(objectPtr){
                    m_fiducialPointsPtr.push_back(objectPtr);

                    // Create visual green sphere
                    cShapeSphere* visualMesh = new cShapeSphere(0.001);
                    visualMesh->setRadius(0.002);
                    visualMesh->m_material->setRed();
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
            
            
            // ModelMap: map<string, afModelPtr>
            afModelMap* map = m_worldPtr->getModelMap();
            for (auto it=map->begin(); it != map->end(); it++){
                //ChildrenMap: map<map<afType, map<string, afBaseObject*> >
                afChildrenMap* childrenMap = it->second->getChildrenMap();
                for(auto cIt = childrenMap->begin(); cIt != childrenMap->end(); ++cIt){   
                    for (auto it_child=cIt->second.begin(); it_child != cIt->second.end(); ++it_child){
                        // it_child->first /ambf/env/BODY rigidbodyName
                        if (it_child->first.find(node["pointer"]["object name"].as<string>()) != string::npos){
                            m_registeringObjectADFFilePath = it->second->getAttributes()->m_filePath.c_str();
                        }
                    }
                }
            }
            
            // If ADF file was not found then return error
            if (m_registeringObjectADFFilePath.empty()){
                cerr << "ERROR! No ADF file found for the object named " << node["pointer"]["object name"].as<string>() << endl;
                return -1;
            }

            if(!m_registeringObject){
                cerr << "ERROR! No object named " << node["pointer"]["object name"].as<string>() << " found." << endl;
                
                return -1;
            }
            else{
                // Get pointer to camera
                m_modelCamera = m_worldPtr->getCamera("model_camera");
                m_panelManager.addCamera(m_modelCamera);

                if (m_modelCamera){
                    // Set background
                    cBackground* background = new cBackground();
                    background->setCornerColors(cColorf(0.2f, 0.2f, 0.2f),
                                                cColorf(0.2f, 0.2f, 0.2f),
                                                cColorf(0.2f, 0.2f, 0.2f),
                                                cColorf(0.2f, 0.2f, 0.2f));
                    m_modelCamera->getBackLayer()->addChild(background);

                    // Load Camera related parameters
                    if (node["pointer"]["camera"].IsDefined()){
                        YAML::Node camLocationNode = node["pointer"]["camera"]["location_offset"];
                        m_camLocationOffset = to_cVector3d(adf_loader_1_0::ADFUtils::positionFromNode(&camLocationNode));
                        YAML::Node camLookAtNode = node["pointer"]["camera"]["look at"];
                        m_camLookAtOffset = to_cVector3d(adf_loader_1_0::ADFUtils::positionFromNode(&camLookAtNode));
                        YAML::Node camUpNode = node["pointer"]["camera"]["up"];
                        m_camUp = to_cVector3d(adf_loader_1_0::ADFUtils::positionFromNode(&camUpNode));
                    }
                    else{
                        m_camLocationOffset = 0.5 * cVector3d(0, 1.0, 0.0);
                        m_camLookAtOffset = cVector3d(0, -1.0, 0.0);
                        m_camUp = cVector3d(0, 0.0, 1.0);
                    }

                    // Set camera for the first time
                    cVector3d initCamPos = m_registeringObject->getLocalPos() + m_camLocationOffset;
                    cVector3d initLookAt = m_registeringObject->getLocalPos() + m_camLookAtOffset;
                    m_modelCamera->setView(initCamPos, initLookAt, m_camUp);

                    // Set light at the same location
                    m_modelLight = m_worldPtr->getLight("model_light");
                    m_modelLight->setLocalPos(initCamPos);
                    m_modelLight->setDir(initLookAt - initCamPos);
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
            m_pivotMarkerName = markerName;
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

            // [Optional] TF in the tf config (--tf_list) written with [Ctrl + W]. Default: FIXED transform from the marker
            if(node["pivot"]["tf name"]){
                m_pivotTFName = node["pivot"]["tf name"].as<string>();
            }

            if(node["pivot"]["registered pivot result"]){
                double x = node["pivot"]["registered pivot result"]["t_tip"]["x"].as<double>();
                double y = node["pivot"]["registered pivot result"]["t_tip"]["y"].as<double>();
                double z = node["pivot"]["registered pivot result"]["t_tip"]["z"].as<double>();

                m_marker2tip.set(x, y, z);
                m_pivotDefined = true;
            }
        }

        if (node["axis"]){
            cout << "> Axis Calibration" << endl;
            m_isAxis = true;

            // Setting up CRTK communication
            string marker_nspace = node["axis"]["marker namespace"].as<string>();
            m_axisToolInterface = new CRTKInterface(marker_nspace);

            if (node["axis"]["reference namespace"]){
                m_axisReferenceInterface = new CRTKInterface(node["axis"]["reference namespace"].as<string>());
            }

            if(node["axis"]["resolution"]){
                m_axisRes = node["axis"]["resolution"].as<double>();
            }

            if(node["axis"]["number of points"]){
                m_numAxis = node["axis"]["number of points"].as<int>();
            }

            // Tool tip w.r.t. marker. Falls back to the pivot calibration result if not specified.
            if(node["axis"]["t_tip"]){
                YAML::Node tipNode = node["axis"]["t_tip"];
                m_axisMarker2tip = to_cVector3d(adf_loader_1_0::ADFUtils::positionFromNode(&tipNode));
                m_axisTipDefined = true;
            }

            // Axis of the body mesh (local frame) that runs along the shaft, from the tip to the back of the tool
            if(node["axis"]["mesh axis"]){
                YAML::Node meshAxisNode = node["axis"]["mesh axis"];
                m_axisMeshAxis = to_cVector3d(adf_loader_1_0::ADFUtils::positionFromNode(&meshAxisNode));
                if (m_axisMeshAxis.length() < 1e-9){
                    cerr << "ERROR! 'mesh axis' can not be zero" << endl;
                    return -1;
                }
                m_axisMeshAxis.normalize();
            }

            // Rotation of the mesh about the shaft axis [rad]
            if(node["axis"]["roll offset"]){
                m_axisRollOffset = node["axis"]["roll offset"].as<double>();
            }

            // Marker body in AMBF, used to find the tip in the tf config and to show the calibrated axis
            if(node["axis"]["marker name"]){
                m_pivotMarkerName = node["axis"]["marker name"].as<string>();
                m_pivotMarkerPtr = m_worldPtr->getRigidBody(m_pivotMarkerName);
            }

            // [Optional] TF in the tf config (--tf_list) written with [Ctrl + W]. Default: FIXED transform from the tip
            if(node["axis"]["tf name"]){
                m_axisTFName = node["axis"]["tf name"].as<string>();
            }
        }
        // Without an axis section, enable axis calibration with the pivot marker and default parameters
        else if (m_isPivot){
            cout << "> Axis Calibration (using pivot marker namespace and default parameters)" << endl;
            m_isAxis = true;
            m_axisToolInterface = m_pivotToolInterface;
            m_axisReferenceInterface = m_pivotReferenceInterface;
        }

        // Tool tip for axis calibration: axis section > pivot result > FIXED transform from the marker in the tf config
        if (m_isAxis && !m_axisTipDefined && !m_pivotDefined){
            if (readTipFromTFConfig(m_axisMarker2tip)){
                m_axisTipDefined = true;
            }
            else{
                cerr << "WARNING! No 't_tip' for axis calibration. Run pivot calibration first in this session "
                     << "or the axis sign will be ambiguous." << endl;
            }
        }
        cerr << "SUCCESSFULLY Initialized plugin!!" << endl;

        return 1;

}

// TF of the tf config (--tf_list) that holds the tool tip w.r.t. the marker (ex. TF marker_body-drill_tip)
string afRegistrationPlugin::getTFTipName(string& tipChild){
    tipChild.clear();
    if (m_tfConfigFilepath.empty()){
        return "";
    }
    if (!m_pivotTFName.empty()){
        try{
            YAML::Node tf = YAML::LoadFile(m_tfConfigFilepath)[m_pivotTFName];
            if (tf && tf["child"]){
                tipChild = tf["child"].as<string>();
            }
        }
        catch (const exception& e){
            cerr << "WARNING! Failed to read tf config " << m_tfConfigFilepath << ": " << e.what() << endl;
        }
        return m_pivotTFName;
    }
    if (m_pivotMarkerName.empty()){
        return "";
    }
    return TFConfigWriter::findFixedTransform(m_tfConfigFilepath, m_pivotMarkerName, tipChild);
}

bool afRegistrationPlugin::readTipFromTFConfig(cVector3d& marker2tip){
    string tipChild;
    string tfName = getTFTipName(tipChild);
    if (tfName.empty()){
        return false;
    }
    try{
        YAML::Node posNode = YAML::LoadFile(m_tfConfigFilepath)[tfName]["transformation"]["position"];
        if (!posNode){
            return false;
        }
        marker2tip = to_cVector3d(adf_loader_1_0::ADFUtils::positionFromNode(&posNode));
        cout << "> Axis calibration t_tip from '" << tfName << "' in " << m_tfConfigFilepath << ": " << marker2tip.str(6) << endl;
        return true;
    }
    catch (const exception& e){
        cerr << "WARNING! Failed to read " << tfName << " in " << m_tfConfigFilepath << ": " << e.what() << endl;
    }
    return false;
}

string afRegistrationPlugin::savePromptText(){
    if (m_tfConfigFilepath.empty()){
        return "No tf config (--tf_list) given: copy the result manually.";
    }
    string name = (m_activeMode == RegistrationMode::PIVOT) ? "pivot" : "axis";
    return "Save " + name + " result to tf config? Press [Ctrl + W]\n(" + m_tfConfigFilepath + ")\n"
           "Only this result is saved.";
}

void afRegistrationPlugin::restartCalibration(){
    if (m_activeMode == RegistrationMode::PIVOT){
        for (auto mesh: m_pivotPointMeshes){
            mesh->setShowEnabled(false);
        }
        m_pivotPointMeshes.clear();
        m_savedPivotPoints.clear();
        m_savedPivotRef2Points.clear();
        m_pivotDone = false;
        m_pivotText.clear();
        cerr << "Pivot calibration restarted." << endl;
    }
    else if (m_activeMode == RegistrationMode::AXIS){
        for (auto mesh: m_axisPointMeshes){
            mesh->setShowEnabled(false);
        }
        m_axisPointMeshes.clear();
        m_savedAxisPoints.clear();
        m_axisDone = false;
        m_axisText.clear();
        if (m_axisLine){
            m_axisLine->setShowEnabled(false);
        }
        cerr << "Axis calibration restarted." << endl;
    }
    m_tfSaveText.clear();
}

// Write the pivot (tip position) or axis (body orientation) result into the tf config (--tf_list)
void afRegistrationPlugin::saveCalibrationToTF(){
    if (m_tfConfigFilepath.empty()){
        m_tfSaveText = "ERROR! No tf config (--tf_list) given.";
        cerr << m_tfSaveText << endl;
        return;
    }

    string tipChild;
    string tipTF = getTFTipName(tipChild);
    string date = TFConfigWriter::timestamp("%Y-%m-%d %H:%M:%S");
    string tfName, header, errorMsg, backupPath;
    vector<pair<string, string>> values;
    char buf[256];

    if (m_activeMode == RegistrationMode::PIVOT){
        if (m_manualPivot){
            m_tfSaveText = "ERROR! Saving a MANUAL pivot calibration to the tf config is not supported.";
            cerr << m_tfSaveText << endl;
            return;
        }
        tfName = tipTF;
        if (tfName.empty()){
            m_tfSaveText = "ERROR! No FIXED transform with parent '" + m_pivotMarkerName + "' in the tf config.\nSet 'tf name' in the pivot section.";
            cerr << m_tfSaveText << endl;
            return;
        }
        snprintf(buf, sizeof(buf), "{x: %.7g, y: %.7g, z: %.7g}", m_marker2tip.x(), m_marker2tip.y(), m_marker2tip.z());
        values.push_back(make_pair(string("position"), string(buf)));
        snprintf(buf, sizeof(buf), "%.6g", m_pivotMeanError);
        values.push_back(make_pair(string("Mean calibration error"), string(buf)));
        header = "Pivot calibration performed on " + date + " (ambf_registration_plugin)";
    }
    else if (m_activeMode == RegistrationMode::AXIS){
        tfName = m_axisTFName;
        if (tfName.empty() && !tipChild.empty()){
            string bodyChild;
            tfName = TFConfigWriter::findFixedTransform(m_tfConfigFilepath, tipChild, bodyChild);
        }
        if (tfName.empty()){
            m_tfSaveText = "ERROR! No FIXED transform from the tool tip in the tf config.\nSet 'tf name' in the axis section.";
            cerr << m_tfSaveText << endl;
            return;
        }

        // The axis result is w.r.t. the marker. Express it w.r.t. the tip frame: R_tip_body = R_marker_tip^T * R_marker_body
        cMatrix3d markerToTip;
        markerToTip.identity();
        double r, p, y;
        if (!tipTF.empty() && TFConfigWriter::readOrientation(m_tfConfigFilepath, tipTF, r, p, y)){
            afMatrix3d rot;
            rot.setRPY(r, p, y);
            markerToTip = to_cMatrix3d(rot);
        }
        cMatrix3d tipToMarker, tipToBody;
        markerToTip.transr(tipToMarker);
        tipToMarker.mulr(m_axisResult.markerToBodyRot, tipToBody);
        double yaw = atan2(tipToBody(1, 0), tipToBody(0, 0));
        double pitch = atan2(-tipToBody(2, 0), sqrt(tipToBody(2, 1) * tipToBody(2, 1) + tipToBody(2, 2) * tipToBody(2, 2)));
        double roll = atan2(tipToBody(2, 1), tipToBody(2, 2));

        snprintf(buf, sizeof(buf), "{r: %.6f, p: %.6f, y: %.6f}", roll, pitch, yaw);
        values.push_back(make_pair(string("orientation"), string(buf)));
        snprintf(buf, sizeof(buf), " (coverage: %.1f deg, axis spread: %.2f deg)",
                 m_axisResult.angularCoverage * 180.0 / M_PI, m_axisResult.axisSpread * 180.0 / M_PI);
        header = "Axis calibration performed on " + date + buf;
    }
    else{
        return;
    }

    if (TFConfigWriter::updateTransformation(m_tfConfigFilepath, tfName, values, header, backupPath, errorMsg)){
        m_tfSaveText = "Saved to '" + tfName + "' in " + m_tfConfigFilepath + "\nBackup: " + backupPath +
                       "\nRestart the simulation to load the new tf config.";
        cout << "[INFO!] " << m_tfSaveText << endl;
    }
    else{
        m_tfSaveText = "ERROR! " + errorMsg;
        cerr << m_tfSaveText << endl;
    }
}

void afRegistrationPlugin::reset(){
    cerr << "INFO! PLUGIN RESET CALLED" << endl;
}

bool afRegistrationPlugin::close(){
    delete m_registrationStatusLabel;
    delete m_savedPointsListLabel;
    return -1;
}

void afRegistrationPlugin::saveDataToCSV(string fileName, vector<cTransform> vecTransform){
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

void afRegistrationPlugin::reWriteADFfile(string filePath, btTransform registeredTransform){
    // Prompt if you want to rewrite the ADF file with the registered transform
    // cout << " >> Do you want to rewrite the ADF file with the registered transform? (y/n)" << endl;
    // char answer;
    // cin >> answer; 

    // if (answer == 'y' || answer == 'Y'){
        
    // Load the ADF file
    YAML::Node adfNode = YAML::LoadFile(filePath);
    string rawData = m_registeringObject->getAttributes()->getSpecificationData().m_rawData; // Get the name of the object to be registered

    // objectName is between name: and the next line break in the raw data
    size_t namePos = rawData.find("name:") + 6; // Move the position to the end of "name:"
    size_t endLinePos = rawData.find("\n", namePos); // Find the position of the next line break after the name
    string objectName = rawData.substr(namePos, endLinePos - namePos); // Extract the object name
    // objectName.erase(remove_if(objectName.begin(), objectName.end(), ::isspace), objectName.end()); // Remove any whitespace from the object name        

    YAML::Node objectNode;
    cerr << "Searching for the object named " << objectName << " in the ADF file..." << endl;
    for (int i=0; i < adfNode["bodies"].size(); i++){
        string rigidbodyName = adfNode["bodies"][i].as<string>();
        cerr << "Checking object: " << adfNode[rigidbodyName]["name"].as<string>() << endl;
        if (adfNode[rigidbodyName]["name"].as<string>() == objectName){
            // Found the object node, break the loop
            objectNode = adfNode[rigidbodyName]["location"];
            cerr << "Object node found in ADF file." << endl;
            break;
        }
    }

    if (objectNode.IsNull()){
        cerr << "ERROR! No object named " << objectName << " found in the ADF file." << endl;
        return;
    }

    cerr << "Object found in ADF file: " << objectName << endl;
    cerr << objectNode["position"]["x"] << ", " << objectNode["position"]["y"] << ", " << objectNode["position"]["z"] << endl;

    // Move the object to the registered location
    btTransform currentTransform = m_registeringObject->m_bulletRigidBody->getWorldTransform();

    // Update position
    const btVector3& pos = currentTransform.getOrigin();

    objectNode["position"]["x"] = static_cast<double>(pos.x());
    objectNode["position"]["y"] = static_cast<double>(pos.y());
    objectNode["position"]["z"] = static_cast<double>(pos.z());

    // Convert rotation matrix to roll, pitch, yaw
    double yaw, pitch, roll;
    currentTransform.getBasis().getEulerYPR(yaw, pitch, roll);

    // Update orientation
    objectNode["orientation"]["r"] = roll;
    objectNode["orientation"]["p"] = pitch;
    objectNode["orientation"]["y"] = yaw;

    // Rename the old ADF file with timestamp
    // YYYYMMDD_HHMMSS format
    std::time_t now = std::time(nullptr);
    std::tm tm_now;
    localtime_r(&now, &tm_now);

    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S");
    string newFilePath = filePath.substr(0, filePath.find_last_of(".")) + ".yaml." + oss.str() + ".old";
    rename(filePath.c_str(), newFilePath.c_str());
    cout << "[INFO!] ADF file rewritten. Old file is renamed to: " << newFilePath << endl;

    m_registeredText = "Registration saved!!\nOld file is renamed to: " + newFilePath;

    m_isADFsaved = true;

    // Save the updated ADF file
    std::ofstream fout(filePath);
    fout << adfNode;
    fout.close();
}
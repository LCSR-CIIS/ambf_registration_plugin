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
                    cerr << "WARNING! No object named " << node["pointer"]["name of points"][i].as<string>() << " found." << endl;
                }
            }

            cerr << m_numPoints << "Points are specified as Keypoints" << endl;
        }

        // Check whether optical tracker based registration is needed or not
        if (node["optical tracker"]){
            cout << "Optical Tracker based registration" << endl;

            string nspace = node["optical tracker"]["namespace"].as<string>();
            m_numPoints = node["pointer"]["name of points"].size();
            
            // Load alll the points
            for (int i=0; i < m_numPoints; i++){

                afRigidBodyPtr objectPtr;
                string objectName = node["pointer"]["name of points"][i].as<string>();
                objectPtr = m_worldPtr->getRigidBody(objectName);
                
                if(objectPtr){
                    m_trackingPointsPtr.push_back(objectPtr);
                    CRTKInterface* interface = new CRTKInterface(nspace + "/" + objectName);
                    m_trackingPoints.push_back(interface);
                }
                else{
                    cerr << "WARNING! No object named " << objectName << "found." << endl;
                }
            }
        }
    }

    else{
        cerr << "ERROR! NO configuration file specified." << endl;
        return -1;
    }

    return 1;
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
            m_activeMode = RegistrationMode::POINTER;
            cerr << "Registration Mode changed to POINTER " << endl;
        }

        else if (a_key == GLFW_KEY_2){
            m_activeMode = RegistrationMode::TRACKER;
            cerr << "Registration Mode changed to TRACKER " << endl;
        }
        
        // Need to remove later but assigning it for testing
        else if (a_key == GLFW_KEY_3){
            m_activeMode = RegistrationMode::REGISTERED;
            cerr << "Registration Mode changed to REGISTERED " << endl;
        }

        else if(a_key == GLFW_KEY_4){
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
            break;

        case RegistrationMode::REGISTERED:
            m_panelManager.setText(m_registrationStatusLabel, "Registration Status: REGISTERED");
            m_panelManager.setFontColor(m_registrationStatusLabel, cColorf(0.,0.5,0.5));
            break;
    }

    m_panelManager.update();

}

void afRegistrationPlugin::physicsUpdate(double dt){
    if (m_activeMode == RegistrationMode::POINTER){
        if (m_savePoint){
            cShapeSphere* pointMesh = new cShapeSphere(0.001);
            pointMesh->setRadius(0.001);
            pointMesh->m_material->setBlack();
            pointMesh->m_material->setShininess(0);
            pointMesh->m_material->m_specular.set(0, 0, 0);
            pointMesh->setShowEnabled(true);
            pointMesh->setLocalPos(m_toolTipPtr->getLocalPos());
            m_worldPtr->addSceneObjectToWorld(pointMesh);
            m_spheres.push_back(pointMesh);

            m_savePoint = false;
        }

        // Once all the points are saved
        if (m_spheres.size() == m_pointsPtr.size()){
            cerr << "Moving all the collected points" << endl;
            for (int idx=0; idx<m_spheres.size(); idx++){
                m_pointsPtr[idx]->setLocalPos(m_spheres[idx]->getLocalPos());
            }
        }
    }

    else if (m_activeMode == RegistrationMode::TRACKER){
        for (int i=0; i< m_numPoints; i++){
            if(m_pointsPtr[i]){

                // Retreive point transformation from the rostopic
                // cTransform trans = m_trackingPoints[i]->measured_cp();

                // if (abs(trans.getLocalPos().x())>=0.0){
                //     // TODO: Need to change here
                //     m_pointsPtr[i]->setLocalTransform(trans);
                // }
            }
        }
    }

    else if (m_activeMode == RegistrationMode::REGISTERED){

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


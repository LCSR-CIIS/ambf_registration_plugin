
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

#include "test_plugin.h"
using namespace std;

afTestPlugin::afTestPlugin(){
    cout << "/*********************************************" << endl;
    cout << "/* AMBF Plugin for Testing" << endl;
    cout << "/*********************************************" << endl;
}

int afTestPlugin::init(int argc, char** argv, const afWorldPtr a_afWorld){
    p_opt::options_description cmd_opts("Command Line Options");
    cmd_opts.add_options()
            ("info", "Show Info")
            ("config", p_opt::value<string>()->default_value(""), "path to config file");

    p_opt::variables_map var_map;
    p_opt::store(p_opt::command_line_parser(argc, argv).options(cmd_opts).allow_unregistered().run(), var_map);
    p_opt::notify(var_map);

    if(var_map.count("info")){
        cout<< cmd_opts << endl;
        return -1;
    }
    // Loading options 
    string config_path = var_map["config"].as<string>();

    // Initialize Camera
    m_worldPtr = a_afWorld;

    // Improve the constratint
    m_worldPtr->m_bulletWorld->getSolverInfo().m_erp = 1.0;  // improve out of plane error of joints
    m_worldPtr->m_bulletWorld->getSolverInfo().m_erp2 = 1.0; // improve out of plane error of joints

    int result = readConfigFile(config_path);

    // Add shphere at the location of the rigidBody
    addSphereInWorld(m_rigidBody->getLocalPos());

    return 1;
}


// Define Key board shortcuts
void afTestPlugin::keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods){
    if (a_key == GLFW_KEY_0){
        m_keyPressed = true;
    }

    if (a_key == GLFW_KEY_2){
        m_isVisible = !m_isVisible; 
        m_rigidBody->setVisibleFlag(m_isVisible);
    }
}

// Graphics related updates
void afTestPlugin::graphicsUpdate(){
}

void afTestPlugin::applyTransformToRigidBody(afRigidBodyPtr bodyPtr, cTransform& trans){
    
    // Retreive the current location
    btTransform currentTransform, Tcommand;
    bodyPtr->m_bulletRigidBody->getMotionState()->getWorldTransform(currentTransform);

    // Apply transformation
    Tcommand.mult(currentTransform, to_btTransform(trans)); // Correct
    // Tcommand.mult(to_btTransform(trans), currentTransform); // Incorrect 

    bodyPtr->m_bulletRigidBody->getMotionState()->setWorldTransform(Tcommand);
    bodyPtr->m_bulletRigidBody->setWorldTransform(Tcommand);
    
    // bodyPtr->setLocalTransform(to_cTransform(Tcommand));
    
}

void afTestPlugin::addSphereInWorld(cVector3d pos){
    cShapeSphere* point = new cShapeSphere(0.001);
    point->setRadius(0.001);
    point->m_material->setBlue();
    point->m_material->setShininess(0);
    point->m_material->m_specular.set(0, 0, 0);
    point->setShowEnabled(true);
    point->setLocalPos(pos);
    m_worldPtr->addSceneObjectToWorld(point);
}

// Physics related updates
void afTestPlugin::physicsUpdate(double dt){
    if (m_keyPressed && m_numKeyPressed < m_transformList.size()){
        
        cout << "Applying Transformation \n" << m_transformList[m_numKeyPressed].getLocalPos().str(5) + "\n" << m_transformList[m_numKeyPressed].getLocalRot().str(5) << endl;
        cTransform currentTrans = m_rigidBody->getLocalTransform();
        cTransform trans;
        // m_transformList[m_numKeyPressed].mulr(currentTrans, trans); // Inorrect 
        currentTrans.mulr(m_transformList[m_numKeyPressed], trans); // Correct??

        addSphereInWorld(trans.getLocalPos());
        applyTransformToRigidBody(m_rigidBody, m_transformList[m_numKeyPressed]);

        m_keyPressed = false;
        m_numKeyPressed += 1;
    }
}

int afTestPlugin::readConfigFile(string config_filepath){
    cerr << "> loading the user defined configuration file..." << endl; 
    cerr << config_filepath << endl;

    //Load the user defined object here. 
    YAML::Node node = YAML::LoadFile(config_filepath);

    string rigidBodyName = node["rigidbody"].as<string>();
    m_rigidBody = m_worldPtr->getRigidBody(rigidBodyName);

    for (size_t i = 0; i < node["transformations"].size(); i++){
        string transformName = node["transformations"][i].as<string>();
        
        if (node[transformName]){
            YAML::Node transformPos = node[transformName]["transformation"]["position"];
            YAML::Node transformOri = node[transformName]["transformation"]["orientation"];

            cVector3d trans = to_cVector3d(adf_loader_1_0::ADFUtils::positionFromNode(&transformPos));
            cMatrix3d rot = to_cMatrix3d(adf_loader_1_0::ADFUtils::rotationFromNode(&transformOri));

            cTransform transform;
            transform.setLocalPos(trans);
            transform.setLocalRot(rot);

            m_transformList.push_back(transform);
        }
    }
}

void afTestPlugin::reset(){
    cerr << "INFO! PLUGIN RESET CALLED" << endl;
}

bool afTestPlugin::close(){
    return -1;
}
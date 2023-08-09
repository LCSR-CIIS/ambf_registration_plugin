//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2022, AMBF
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


#include "point_cloud_registration.h"

PointCloudRegistration::PointCloudRegistration(){
}

int PointCloudRegistration::ICPRegistration(vector<cVector3d> pointsIn, vector<cVector3d> pointsOut, btTransform &trans)
{   
    if (pointsIn.size() != pointsOut.size()){
        cerr << "ERROR! The size of the input poin cloud does not match with the source point cloud." << endl;
        return -1;
    }

    // Convert the Points to the Pcl::pointXYZ data type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>(pointsIn.size(),1));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZ>(pointsIn.size(),1));
    cvectorToPointCloud(pointsIn, cloudIn);
    cvectorToPointCloud(pointsOut, cloudOut);

    // Perform ICP registration
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloudIn);
    icp.setInputTarget(cloudOut);
    
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;
    cout << icp.getFinalTransformation() << endl;

    Eigen::Matrix<float, 4, 4> Trans;
    Trans = icp.getFinalTransformation();
    
    // Convert Eigen Matrix4 to btTransform
    eigenMatrixTobtTransform(Trans, trans);

    return 1;
}  

void PointCloudRegistration::cvectorToPointCloud(vector<cVector3d> points,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    for (size_t i=0; i < points.size(); i++){
        cloud->points[i].x = points[i].x();  
        cloud->points[i].y = points[i].y();  
        cloud->points[i].z = points[i].z();  
    }
}

void PointCloudRegistration::eigenMatrixTocTransform(Eigen::Matrix<float, 4, 4> Trans, cTransform &trans){
    // Chai3d related dataType
    cVector3d chai_T(Trans(0,3), Trans(1,3), Trans(2,3));
    cMatrix3d chai_R(Trans(0,0),Trans(1,0),Trans(2,0), Trans(0,1), Trans(1,1), Trans(2,1), Trans(0,2), Trans(1,2), Trans(2,1));
    
    trans.setLocalPos(chai_T);
    trans.setLocalRot(chai_R);
}

void PointCloudRegistration::eigenMatrixTobtTransform(Eigen::Matrix<float, 4, 4> Trans, btTransform &trans){
    
    // Converted to Chai3d data type first
    cTransform tmp_trans;
    eigenMatrixTocTransform(Trans, tmp_trans);

    // Convert Rotation into quaternion
    cQuaternion chai_qr;
    chai_qr.fromRotMat(tmp_trans.getLocalRot());

    // Change to btVector and Matrix
    btVector3 btTrans;
    btTrans.setValue(tmp_trans.getLocalPos().x(), tmp_trans.getLocalPos().y(), tmp_trans.getLocalPos().z());   
    btQuaternion btRot(chai_qr.x,chai_qr.y, chai_qr.z, chai_qr.w);
    trans.setOrigin(btTrans);
    trans.setRotation(btRot);
}
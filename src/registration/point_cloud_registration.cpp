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
    icp.setMaximumIterations(1);

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

int PointCloudRegistration::PointSetRegistration(vector<cVector3d> &pointsIn, vector<cVector3d> &pointsOut, btTransform &trans, vector<cVector3d> &newPoints){
    if (pointsIn.size() != pointsOut.size()){
        cerr << "ERROR! The size of the input poin cloud does not match with the source point cloud." << endl;
        return -1;
    }

    if (0){
        cerr << "Using the predefined points:" << endl;
        vector<cVector3d> In;
        In.push_back(cVector3d(0.05265, 0.05593, 0.12162));
        In.push_back(cVector3d(0.05156, 0.05592, 0.05774));
        In.push_back(cVector3d(-0.00250, 0.05485, 0.05883));
        In.push_back(cVector3d(-0.00273, 0.05486, 0.12340));

        vector<cVector3d> Out;
        Out.push_back(cVector3d(0.16867, -0.03044, -0.65534));
        Out.push_back(cVector3d(0.23257, -0.03034, -0.65667));
        Out.push_back(cVector3d(0.23346, -0.08365, -0.65663));
        Out.push_back(cVector3d(0.16971, -0.08453, -0.65422));

        pointsIn = In;
        pointsOut = Out;
    }

    
    // 1. Get the center of the each point and align them
    vector<Eigen::Vector3d> vecPointIn;
    vector<Eigen::Vector3d> vecPointOut;

    Eigen::Vector3d aveIn;
    Eigen::Vector3d aveOut;

    for (size_t i = 0; i < pointsIn.size(); i++){
        cerr << "Points In:" << pointsIn[i].str(5) << endl;
        aveIn += Eigen::Vector3d(pointsIn[i].x(), pointsIn[i].y(), pointsIn[i].z());
        aveOut += Eigen::Vector3d(pointsOut[i].x(), pointsOut[i].y(), pointsOut[i].z());
        vecPointIn.push_back(Eigen::Vector3d(pointsIn[i].x(), pointsIn[i].y(), pointsIn[i].z()));
        vecPointOut.push_back(Eigen::Vector3d(pointsOut[i].x(), pointsOut[i].y(), pointsOut[i].z()));
    }

    for (size_t i=0; i< pointsIn.size(); i++){
        cerr << "Points Out:" << pointsOut[i].str(5) << endl;
    }

    aveIn *= 1.0/pointsIn.size();
    aveOut *= 1.0/pointsOut.size();

    for (size_t i = 0; i < pointsIn.size(); i++){
        vecPointIn[i] = vecPointIn[i] - aveIn;
        vecPointOut[i] = vecPointOut[i] - aveOut;
    }

    cerr << "aveIn: " << endl;
    cerr << aveIn << endl;
    cerr << "aveOut: " << endl;
    cerr << aveOut << endl;

    // 2. Get W = sum(x' p')
    Eigen::MatrixXd W;
    W.setZero(3,3);
    for (size_t i = 0; i < pointsIn.size(); i++){
        W += vecPointIn[i] * vecPointOut[i].transpose();
    }
    
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose(); // Wrong!!
    Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose(); // Correct!!
    cerr << "Rotational det:" << R.determinant() << endl;

    if  (R.determinant() == -1){
        return -1;
    }

    Eigen::Vector3d T = aveOut - R * aveIn;
    cerr << "Rotation Result: " << endl;
    cerr << R << endl;
    cerr << "Euler Angle" << endl;
    cerr << R.eulerAngles(0,1,2) << endl;
    cerr << R.eulerAngles(0,2,1) << endl;
    cerr << R.eulerAngles(1,0,2) << endl;
    cerr << R.eulerAngles(1,2,0) << endl;
    cerr << R.eulerAngles(2,1,0) << endl;
    cerr << R.eulerAngles(2,0,1) << endl;
    cerr << "Translation Result: " << endl;
    cerr <<  T << endl;

    Eigen::Vector3d err;
    for (size_t i = 0; i < pointsIn.size(); i++){
        err += vecPointOut[i] - R * vecPointIn[i];
    }
    cerr << "Error: " << err << endl;
    
    for (size_t i = 0; i < pointsIn.size(); i++){
        Eigen::Vector3d newPointseigen;
        newPointseigen = R * (vecPointIn[i] + aveIn) + T;
        newPoints.push_back(cVector3d(newPointseigen.x(), newPointseigen.y(), newPointseigen.z()));

    }
    Eigen::Affine3d aff = Eigen::Affine3d::Identity();
    aff.translation() = T;
    aff.linear() = R;
    Eigen::Matrix<float, 4, 4> Trans = aff.matrix().cast <float> (); ;
    eigenMatrixTobtTransform(Trans, trans);


    return 1;
}

// int PointCloudRegistration::PointSetRegistration(vector<cVector3d> pointsIn, vector<cVector3d> pointsOut, btTransform &trans){
//     if (pointsIn.size() != pointsOut.size()){
//         cerr << "ERROR! The size of the input poin cloud does not match with the source point cloud." << endl;
//         return -1;
//     }
    
//     // 1. Get the center of the each point and align them
//     // vector<cv::Point3f> vecPointIn;
//     // vector<cv::Point3f> vecPointOut;

//     cv::Mat vecPointIn(1, 4, CV_32FC3);
//     cv::Mat vecPointOut(1, 4, CV_32FC3);

//     for (size_t i = 0; i < pointsIn.size(); i++){
//         vecPointIn.ptr<cv::Point3f>()[i] = cv::Point3f(pointsIn[i].x(),pointsIn[i].y(),pointsIn[i].z());
//         vecPointOut.ptr<cv::Point3f>()[i] = cv::Point3f(pointsOut[i].x(),pointsOut[i].y(),pointsOut[i].z());
//     }

//     cv::Mat cvaff(3,4,CV_64F); 
//     vector<uchar> inliers;

    
//     cv::estimateAffine3D(vecPointIn, vecPointOut, cvaff, inliers);

//     cerr << cvaff << endl;    
//     Eigen::Matrix<float, 4, 4> Trans; 
//     cv::cv2eigen(cvaff, Trans);
//     eigenMatrixTobtTransform(Trans, trans);


//     return 1;
// }

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

    cerr << "Registration Transform: " << endl;
    cerr << "Translation: " << tmp_trans.getLocalPos().str(6) << endl;
    cerr << "Rotation: " << tmp_trans.getLocalRot().str(6) << endl;

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

// For debugging purpose
int main(){
    cout << "Hello from point_cloud_registration.cpp!" << endl;

    vector<cVector3d> In;
    In.push_back(cVector3d(0.05265, 0.05593, 0.12162));
    In.push_back(cVector3d(0.05156, 0.05592, 0.05774));
    In.push_back(cVector3d(-0.00250, 0.05485, 0.05883));
    In.push_back(cVector3d(-0.00273, 0.05486, 0.12340));
    // In.push_back(cVector3d(3.0, 2.0, 3.0));
    // In.push_back(cVector3d(1.0, 5.0, 3.0));
    // In.push_back(cVector3d(2.0, 4.0, 3.0));

    vector<cVector3d> Out;
    // Out.push_back(cVector3d(0.17722, -0.05181, -0.75032));
    // Out.push_back(cVector3d(0.11920, -0.05530, -0.75301));
    // Out.push_back(cVector3d(0.12326, 0.00385, -0.75118));
    // Out.push_back(cVector3d(0.18432, 0.00502, -0.75278));

    // Out.push_back(cVector3d(0.14747, -0.14746, -0.59895));
    // Out.push_back(cVector3d(0.08167, -0.14540, -0.60467));
    // Out.push_back(cVector3d(0.08188, -0.09239, -0.60251));
    // Out.push_back(cVector3d(0.14230, -0.09493, -0.59773));

    Out.push_back(cVector3d(0.19723, -0.08322, -0.59143));
    Out.push_back(cVector3d(0.13097, -0.08387, -0.59478));
    Out.push_back(cVector3d(0.13078, -0.03173, -0.59273));
    Out.push_back(cVector3d(0.19712, -0.03364, -0.58880));

    cerr << In.size() << "|" << In[0].str(3) << endl;
    PointCloudRegistration pcr;
    btTransform trans;
    // pcr.ICPRegistration(In, Out, trans);
    // pcr.PointSetRegistration(In, Out, trans);
    // cerr << trans.getOrigin().x() << ", " << trans.getOrigin().y() << ", " << trans.getOrigin().z() << endl;

    return 1;
}
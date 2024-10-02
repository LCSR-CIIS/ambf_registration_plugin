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
        return 0;
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
    // eigenMatrixTobtTransform(Trans, trans);

    return 1;
} 

// Function to compute the centroid of a set of points
Eigen::Vector3d PointCloudRegistration::computeCentroid(const std::vector<Eigen::Vector3d> &points) {
    Eigen::Vector3d centroid(0, 0, 0);
    for (const auto &point : points) {
        centroid += point;
    }
    centroid /= points.size();
    return centroid;
}

// Function to apply transformation to a point
Eigen::Vector3d PointCloudRegistration::applyTransformation(const Eigen::Matrix4d &transformation, const Eigen::Vector3d &point) {
    Eigen::Vector4d homogenousPoint(point(0), point(1), point(2), 1.0); // Make it a homogeneous coordinate
    Eigen::Vector4d transformedPoint = transformation * homogenousPoint;
    return transformedPoint.head<3>(); // Return the x, y, z parts
}

// Function to align two sets of points using the point-to-point registration method
int PointCloudRegistration::PointSetRegistration(vector<cVector3d> &pointsIn, vector<cVector3d> &pointsOut, btTransform &btTrans, vector<cVector3d> &estimatedPoints) {
    // 1. Get the center of the each point and align them
    vector<Eigen::Vector3d> sourcePoints;
    vector<Eigen::Vector3d> targetPoints;

    for (size_t i = 0; i < pointsIn.size(); i++){
        sourcePoints.push_back(Eigen::Vector3d(pointsIn[i].x(), pointsIn[i].y(), pointsIn[i].z()));
        targetPoints.push_back(Eigen::Vector3d(pointsOut[i].x(), pointsOut[i].y(), pointsOut[i].z()));
    }

    // Compute centroids of source and target points
    Eigen::Vector3d sourceCentroid = computeCentroid(sourcePoints);
    Eigen::Vector3d targetCentroid = computeCentroid(targetPoints);

    // Subtract centroids to create zero-centered point sets
    std::vector<Eigen::Vector3d> sourceCentered, targetCentered;
    for (size_t i = 0; i < sourcePoints.size(); ++i) {
        sourceCentered.push_back(sourcePoints[i] - sourceCentroid);
        targetCentered.push_back(targetPoints[i] - targetCentroid);
    }

    // Compute covariance matrix
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < sourceCentered.size(); ++i) {
        covariance += sourceCentered[i] * targetCentered[i].transpose();
    }

    // Compute SVD of the covariance matrix
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d rotation = svd.matrixV() * svd.matrixU().transpose();

    // Check for the determinant
    if (rotation.determinant() < 0){
        cerr << "Reflection detected. Fixing the rotation ..." << endl;
        // Flip the last column of matrixV to correct the reflection
        Eigen::Matrix3d V = svd.matrixV();
        V.col(2) = -V.col(2);  // Flip the third column of V
        rotation = V * svd.matrixU().transpose();
    }

    // Compute translation
    Eigen::Vector3d translation = targetCentroid - rotation * sourceCentroid;

    // Build the transformation matrix (4x4 homogeneous matrix)
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) = rotation;
    transformation.block<3, 1>(0, 3) = translation;

    // Convert to bullet
    eigenMatrixTobtTransform(transformation, btTrans);

    // Print the result
    cerr << "position: {x: " << translation[0] << ", y: " << translation[1] << ", z: " << translation[2] << "}" << endl;
    cerr << "orientation: {r: " << rotation.eulerAngles(2,1,0)[2] << ", p: " << rotation.eulerAngles(2,1,0)[1] << ", y: " << rotation.eulerAngles(2,1,0)[0] << "}" << endl;

    // Print Error
    vector<Eigen::Vector3d> transformedPoints;
    for (const auto &point : sourcePoints) {
        transformedPoints.push_back(applyTransformation(transformation, point));
    }

    Eigen::Vector3d totalError = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < transformedPoints.size(); ++i) {
        estimatedPoints.push_back(cVector3d(transformedPoints[i](0),transformedPoints[i](1),transformedPoints[i](2)));
        Eigen::Vector3d errorVector = transformedPoints[i] - targetPoints[i];
        totalError += errorVector.cwiseAbs();  // Accumulate absolute errors in x, y, z directions
    }
    cout << "Total error in x, y, z directions: [" << totalError.transpose() << "]" << std::endl;
    
    return 1;
}


void PointCloudRegistration::cvectorToPointCloud(vector<cVector3d> points,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    for (size_t i=0; i < points.size(); i++){
        cloud->points[i].x = points[i].x();  
        cloud->points[i].y = points[i].y();  
        cloud->points[i].z = points[i].z();  
    }
}

void PointCloudRegistration::eigenMatrixTocTransform(Eigen::Matrix4d& Trans, cTransform &trans){
    // Chai3d related dataType
    cVector3d chai_T(Trans(0,3), Trans(1,3), Trans(2,3));
    cMatrix3d chai_R(Trans(0,0),Trans(1,0),Trans(2,0), Trans(0,1), Trans(1,1), Trans(2,1), Trans(0,2), Trans(1,2), Trans(2,2));
    trans.setLocalPos(chai_T);
    trans.setLocalRot(chai_R);
}

void PointCloudRegistration::convertChaiToBulletTransform(chai3d::cTransform& cTrans, btTransform& btTrans){
    // Set translation
    btVector3 translation;
    translation.setValue(cTrans.getLocalPos().x(), cTrans.getLocalPos().y(), cTrans.getLocalPos().z());

    // Set rotation
    btMatrix3x3 btRotationMatrix;
    btRotationMatrix.setValue(
        cTrans.getLocalRot().getRow(0).x(), cTrans.getLocalRot().getRow(0).y(), cTrans.getLocalRot().getRow(0).z(),
        cTrans.getLocalRot().getRow(1).x(), cTrans.getLocalRot().getRow(1).y(), cTrans.getLocalRot().getRow(1).z(),
        cTrans.getLocalRot().getRow(2).x(), cTrans.getLocalRot().getRow(2).y(), cTrans.getLocalRot().getRow(2).z()
    );

    btTrans.setOrigin(translation);
    btTrans.setBasis(btRotationMatrix);
}
void PointCloudRegistration::eigenMatrixTobtTransform(Eigen::Matrix4d& Trans, btTransform &trans){
    
    // Extract the rotation matrix (top-left 3x3)
    Eigen::Matrix3d eigenRotation = Trans.block<3, 3>(0, 0);

    // Convert Eigen::Matrix3d to btMatrix3x3
    btMatrix3x3 btRotation(
        eigenRotation(0, 0), eigenRotation(0, 1), eigenRotation(0, 2),
        eigenRotation(1, 0), eigenRotation(1, 1), eigenRotation(1, 2),
        eigenRotation(2, 0), eigenRotation(2, 1), eigenRotation(2, 2)
    );

    // Extract the translation vector (last column of the matrix)
    Eigen::Vector3d eigenTranslation = Trans.block<3, 1>(0, 3);

    // Convert Eigen::Vector3d to btVector3
    btVector3 btTranslation(eigenTranslation(0), eigenTranslation(1), eigenTranslation(2));

    // Create the btTransform using the rotation and translation
    btTransform btTransform(btRotation, btTranslation);
    trans = btTransform;
}

// For debugging purpose
int main(){
    cout << "Hello from point_cloud_registration.cpp!" << endl;

    vector<cVector3d> In;
    In.push_back(cVector3d(50.22696175918109, 8.231692163672769, 31.502069387933646));
    In.push_back(cVector3d(51.90740351077355, 27.0052798094135, -5.86047397545072));
    In.push_back(cVector3d(27.212162335048867, 15.287892392436995, -33.555376203331629));
    In.push_back(cVector3d(32.855994893548537, 21.15143555110042, -47.523740918907808));
    In.push_back(cVector3d(21.217579339364997, -40.63420309135226, -28.595321655273439));

    vector<cVector3d> Out;
    Out.push_back(cVector3d(-23.80691248628502, 35.95571905029042, 8.690858961048278));
    Out.push_back(cVector3d(6.7387153977493778, 33.362047891099638, -19.73423526129451));
    Out.push_back(cVector3d(34.13796921107674, 6.482549060020069, -15.175312662720775));
    Out.push_back(cVector3d(46.21870841942769, 10.372429240986998, -25.031828075643995));
    Out.push_back(cVector3d(45.812564336155428, 3.427106906541246, 40.14368713291309));

    cerr << In.size() << "|" << In[0].str(3) << endl;
    PointCloudRegistration pcr;
    btTransform trans;
    vector<cVector3d> newPoints;
    // pcr.ICPRegistration(In, Out, trans);
    pcr.PointSetRegistration(In, Out, trans, newPoints);
    // cerr << trans.getOrigin().x() << ", " << trans.getOrigin().y() << ", " << trans.getOrigin().z() << endl;

    return 1;
}
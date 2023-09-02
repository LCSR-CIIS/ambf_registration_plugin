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


#ifndef POINT_CLOUD_REGISTRATION_H
#define POINT_CLOUD_REGISTRATION_H


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include "chai3d.h"

// For bullet related functions
#include <afFramework.h>

using namespace chai3d;
using namespace std;

class PointCloudRegistration{
    public:
        PointCloudRegistration();
        int ICPRegistration(vector<cVector3d> pointsIn, vector<cVector3d> pointsOut, btTransform& trans);
        int PointSetRegistration(vector<cVector3d> pointsIn, vector<cVector3d> pointsOut, btTransform &trans);
        void cvectorToPointCloud(vector<cVector3d> points,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void eigenMatrixTocTransform(Eigen::Matrix<float, 4, 4> Trans, cTransform &trans);
        void eigenMatrixTobtTransform(Eigen::Matrix<float, 4, 4> Trans, btTransform &trans);
};

int main();

#endif //POINT_CLOUD_REGISTRATION_H
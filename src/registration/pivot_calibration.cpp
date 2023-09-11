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

#include "pivot_calibration.h"

PivotCalibration::PivotCalibration(){

}

Eigen::Vector3d cVectorToEigen(cVector3d vec)
{
    return (Eigen::Vector3d(vec(0), vec(1), vec(2)));
}


Eigen::Matrix3d cMatrixToEigen(cMatrix3d mat)
{
    Eigen::Matrix3d m;
    m(0,0) = mat(0,0);
    m(0,1) = mat(0,1);
    m(0,2) = mat(0,2);
    m(1,0) = mat(1,0);
    m(1,1) = mat(1,1);
    m(1,2) = mat(1,2);
    m(2,0) = mat(2,0);
    m(2,1) = mat(2,1);
    m(2,2) = mat(2,2);
    return (m);
}

int PivotCalibration::calibrate(vector<cTransform> transIn, cVector3d& tipPos, cVector3d& markerPos){
    // Convert to Eigen data type
    vector<Eigen::Matrix3d> vec_R;
    vector<Eigen::Vector3d> vec_P;

    for (cTransform trans: transIn){
        vec_P.push_back(cVectorToEigen(trans.getLocalPos()));
        vec_R.push_back(cMatrixToEigen(trans.getLocalRot()));
    }

	//Calculate tool_tip
	Eigen::MatrixXd A;
	A.resize(vec_R.size() * 3, 6);
	for (int i = 0; i < vec_R.size(); i++)
	{
		A.block(i * 3, 0, 3, 3) = vec_R[i];
		A.block(i * 3, 3, 3, 3) = -vec_R[i].Identity();
	}
	Eigen::VectorXd b;
	b.resize(vec_P.size() * 3);
	for (int i = 0; i < vec_P.size(); i++)
	{
		b.block(i * 3, 0, 3, 1) = -vec_P[i];
	}
	Eigen::MatrixXd x;
	x.resize(6,1);
	// x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b); //after Eigen 3.3
	x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    Eigen::Vector3d tipEigen = x.block(0, 0, 3, 1);
    Eigen::Vector3d markerEigen = x.block(3, 0, 3, 1);
    tipPos.set(tipEigen(0), tipEigen(1), tipEigen(2));

    cerr << "Tip Pose: " << tipPos.str(6) << endl;
    markerPos.set(markerEigen(0), markerEigen(1), markerEigen(2));

    return 1;
}


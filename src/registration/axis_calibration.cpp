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
*/
//==============================================================================

#include "axis_calibration.h"
#include <algorithm>

namespace {
Eigen::Vector3d toEigen(const cVector3d& v){
    return Eigen::Vector3d(v(0), v(1), v(2));
}

Eigen::Matrix3d toEigen(const cMatrix3d& m){
    Eigen::Matrix3d out;
    for (int r = 0; r < 3; r++){
        for (int c = 0; c < 3; c++){
            out(r, c) = m(r, c);
        }
    }
    return out;
}

cVector3d toChai(const Eigen::Vector3d& v){
    return cVector3d(v(0), v(1), v(2));
}
}

AxisCalibration::AxisCalibration(){
}

int AxisCalibration::calibrate(const vector<cTransform>& transIn, const cVector3d* marker2tip,
                               const cVector3d& meshAxis, double rollOffset, AxisCalibrationResult& result){
    const size_t N = transIn.size();
    if (N < 3){
        cerr << "ERROR! Axis calibration needs at least 3 samples, got " << N << endl;
        return -1;
    }

    vector<Eigen::Matrix3d> vec_R;
    vector<Eigen::Vector3d> vec_P;
    for (const cTransform& trans: transIn){
        vec_R.push_back(toEigen(trans.getLocalRot()));
        vec_P.push_back(toEigen(trans.getLocalPos()));
    }

    // 1. Plane fit of the marker origins. The normal is the rotation axis (tracker frame).
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& p: vec_P) centroid += p;
    centroid /= double(N);

    Eigen::MatrixXd A(N, 3);
    for (size_t i = 0; i < N; i++) A.row(i) = (vec_P[i] - centroid).transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
    Eigen::Vector3d u = svd.matrixV().col(0);
    Eigen::Vector3d v = svd.matrixV().col(1);
    Eigen::Vector3d n = svd.matrixV().col(2);

    // 2. Circle fit in the plane (algebraic / Kasa fit): x^2 + y^2 = 2ax + 2by + c
    Eigen::MatrixXd M(N, 3);
    Eigen::VectorXd rhs(N);
    vector<Eigen::Vector2d> pts2d;
    double planeSq = 0.0;
    for (size_t i = 0; i < N; i++){
        Eigen::Vector3d d = vec_P[i] - centroid;
        Eigen::Vector2d q(d.dot(u), d.dot(v));
        pts2d.push_back(q);
        planeSq += pow(d.dot(n), 2);
        M(i, 0) = 2.0 * q.x();
        M(i, 1) = 2.0 * q.y();
        M(i, 2) = 1.0;
        rhs(i) = q.squaredNorm();
    }
    Eigen::Vector3d sol = M.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(rhs);
    Eigen::Vector2d center2d(sol(0), sol(1));
    double radius = sqrt(max(0.0, sol(2) + center2d.squaredNorm()));
    Eigen::Vector3d center = centroid + center2d.x() * u + center2d.y() * v;

    double radiusSq = 0.0;
    vector<double> angles;
    for (const auto& q: pts2d){
        Eigen::Vector2d d = q - center2d;
        radiusSq += pow(d.norm() - radius, 2);
        angles.push_back(atan2(d.y(), d.x()));
    }

    // Angular coverage = 2*pi minus the largest gap between consecutive sample angles
    sort(angles.begin(), angles.end());
    double maxGap = 2.0 * M_PI - (angles.back() - angles.front());
    for (size_t i = 1; i < angles.size(); i++){
        maxGap = max(maxGap, angles[i] - angles[i-1]);
    }

    // 3. Express the axis in the marker frame. It should be constant for a pure rotation about the axis.
    Eigen::Vector3d axisMarker = Eigen::Vector3d::Zero();
    for (const auto& R: vec_R) axisMarker += R.transpose() * n;
    axisMarker.normalize();

    // 4. Orient the axis from the tool tip towards the back of the tool (where the marker is).
    // In the marker frame the marker origin is at 0 and the tip at t_tip, so the axis must point along -t_tip.
    result.signFromTip = false;
    result.tipToAxisDistance = -1.0;
    if (marker2tip){
        Eigen::Vector3d tTip = toEigen(*marker2tip);
        if (axisMarker.dot(-tTip) < 0.0){
            axisMarker = -axisMarker;
            n = -n;
        }
        result.signFromTip = true;

        // The pivot point should lie on the fitted axis
        Eigen::Vector3d pivot = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < N; i++) pivot += vec_R[i] * tTip + vec_P[i];
        pivot /= double(N);
        Eigen::Vector3d w = pivot - center;
        result.tipToAxisDistance = (w - w.dot(n) * n).norm();
    }
    else{
        cerr << "WARNING! No tool tip (t_tip) available. The sign of the axis is ambiguous "
             << "and the mesh could be flipped by 180 degrees." << endl;
    }

    double spread = 0.0;
    for (const auto& R: vec_R){
        double c = max(-1.0, min(1.0, (R.transpose() * n).dot(axisMarker)));
        spread += acos(c);
    }

    // 5. Rotation of the body mesh: shortest rotation taking meshAxis onto axisMarker, then roll about the axis
    Eigen::Vector3d meshAxisE = toEigen(meshAxis).normalized();
    Eigen::Matrix3d Ralign = Eigen::Quaterniond::FromTwoVectors(meshAxisE, axisMarker).toRotationMatrix();
    Eigen::Matrix3d Rbody = Eigen::AngleAxisd(rollOffset, axisMarker).toRotationMatrix() * Ralign;

    // AMBF RPY convention (afMatrix3d::getRPY): R = Rz(yaw) * Ry(pitch) * Rx(roll)
    result.yaw = atan2(Rbody(1, 0), Rbody(0, 0));
    result.pitch = atan2(-Rbody(2, 0), sqrt(Rbody(2, 1) * Rbody(2, 1) + Rbody(2, 2) * Rbody(2, 2)));
    result.roll = atan2(Rbody(2, 1), Rbody(2, 2));

    result.circleCenter = toChai(center);
    result.circleNormal = toChai(n);
    result.circleRadius = radius;
    result.planeRMS = sqrt(planeSq / double(N));
    result.radiusRMS = sqrt(radiusSq / double(N));
    result.angularCoverage = 2.0 * M_PI - maxGap;
    result.axisInMarker = toChai(axisMarker);
    result.axisSpread = spread / double(N);
    for (int r = 0; r < 3; r++){
        for (int c = 0; c < 3; c++){
            result.markerToBodyRot(r, c) = Rbody(r, c);
        }
    }

    cerr << "Axis calibration (" << N << " samples):" << endl;
    cerr << "  Circle center: " << result.circleCenter.str(6) << ", radius: " << radius << endl;
    cerr << "  Plane RMS: " << result.planeRMS << ", radius RMS: " << result.radiusRMS << endl;
    cerr << "  Angular coverage [deg]: " << result.angularCoverage * 180.0 / M_PI << endl;
    cerr << "  Axis spread [deg]: " << result.axisSpread * 180.0 / M_PI << endl;
    if (result.tipToAxisDistance >= 0.0){
        cerr << "  Pivot point to axis distance: " << result.tipToAxisDistance << endl;
    }
    cerr << "  Axis in marker frame (tip -> back): " << result.axisInMarker.str(6) << endl;
    cerr << "Body orientation w.r.t. tip/marker frame:" << endl;
    cerr << "orientation: {r: " << result.roll << ", p: " << result.pitch << ", y: " << result.yaw << "}" << endl;

    return 1;
}

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

#ifndef AXIS_CALIBRATION_H
#define AXIS_CALIBRATION_H

#include <iostream>
#include "chai3d.h"
#include <Eigen/Dense>

using namespace std;
using namespace chai3d;

struct AxisCalibrationResult{
    // Fitted circle traced by the marker origin (tracker/reference frame)
    cVector3d circleCenter;
    cVector3d circleNormal;      // Rotation axis in tracker frame, oriented tip -> back of tool
    double circleRadius = 0.0;
    double planeRMS = 0.0;       // Out-of-plane residual [m]
    double radiusRMS = 0.0;      // In-plane radial residual [m]
    double angularCoverage = 0.0; // Arc swept by the samples [rad]

    // Tool axis expressed in the marker frame, oriented tip -> back of tool
    cVector3d axisInMarker;
    double axisSpread = 0.0;     // Mean angle between per-sample axis estimates and the mean [rad]

    // Distance between the pivot point (from t_tip) and the fitted axis [m]. -1 if t_tip unknown
    double tipToAxisDistance = -1.0;
    bool signFromTip = false;

    // Rotation of the body mesh w.r.t. the marker such that meshAxis is aligned with axisInMarker
    cMatrix3d markerToBodyRot;
    double roll = 0.0, pitch = 0.0, yaw = 0.0; // AMBF convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
};

class AxisCalibration{
    public:
        AxisCalibration();

        // transIn:   marker poses while the tool rotates about its own shaft with the tip resting on a pivot
        // marker2tip: tool tip in the marker frame (pivot calibration result). Used to orient the axis
        //             from the tip towards the back of the tool. Pass nullptr if unknown.
        // meshAxis:  axis of the body mesh (in its local frame) that runs along the shaft, from tip to back
        // rollOffset: extra rotation [rad] of the mesh about the shaft axis
        int calibrate(const vector<cTransform>& transIn, const cVector3d* marker2tip,
                      const cVector3d& meshAxis, double rollOffset, AxisCalibrationResult& result);
};

#endif //AXIS_CALIBRATION_H

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
*/
//==============================================================================

#ifndef TF_CONFIG_WRITER_H
#define TF_CONFIG_WRITER_H

#include <string>
#include <vector>
#include <utility>

using namespace std;

// Edits FIXED transforms of an ambf_tf_plugin config (--tf_list) in place.
// The file is edited line by line so that comments and the history of previous calibrations are kept.
// The replaced values are commented out, and the original file is backed up with a timestamp.
class TFConfigWriter{
    public:
        // Name of the first FIXED transform whose parent is parentName (ex. "TF marker_body-drill_tip").
        // Returns an empty string if none is found. child is set to the child of that transform.
        static string findFixedTransform(const string& filePath, const string& parentName, string& child);

        // Read "transformation: orientation: {r, p, y}" of a transform. Returns false if not defined.
        static bool readOrientation(const string& filePath, const string& tfName, double& r, double& p, double& y);

        // Replace the given keys (ex. "position" -> "{x: 0.1, y: 0.2, z: 0.3}") under "transformation:" of tfName.
        // header is written as a comment above the new values.
        // On success, backupPath is set to the path of the backup of the original file.
        static bool updateTransformation(const string& filePath, const string& tfName,
                                         const vector<pair<string, string>>& values, const string& header,
                                         string& backupPath, string& errorMsg);

        // Current local time, ex. format "%Y%m%d_%H%M%S"
        static string timestamp(const char* format);
};

#endif //TF_CONFIG_WRITER_H

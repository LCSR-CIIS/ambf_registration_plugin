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

#include "tf_config_writer.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <set>

namespace {
size_t indentOf(const string& line){
    size_t n = 0;
    while (n < line.size() && line[n] == ' ') n++;
    return n;
}

bool isCommentOrEmpty(const string& line){
    size_t n = indentOf(line);
    return n == line.size() || line[n] == '#';
}

bool startsWithKey(const string& line, const string& key){
    size_t n = indentOf(line);
    return line.compare(n, key.size() + 1, key + ":") == 0;
}
}

string TFConfigWriter::timestamp(const char* format){
    std::time_t now = std::time(nullptr);
    std::tm tm_now;
    localtime_r(&now, &tm_now);
    std::ostringstream oss;
    oss << std::put_time(&tm_now, format);
    return oss.str();
}

string TFConfigWriter::findFixedTransform(const string& filePath, const string& parentName, string& child){
    try{
        YAML::Node root = YAML::LoadFile(filePath);
        for (size_t i = 0; i < root["transformations"].size(); i++){
            string tfName = root["transformations"][i].as<string>();
            YAML::Node tf = root[tfName];
            if (tf["parent"] && tf["parent"].as<string>() == parentName &&
                tf["type"] && tf["type"].as<string>() == "FIXED"){
                child = tf["child"] ? tf["child"].as<string>() : "";
                return tfName;
            }
        }
    }
    catch (const exception& e){
        cerr << "WARNING! Failed to read tf config " << filePath << ": " << e.what() << endl;
    }
    return "";
}

bool TFConfigWriter::readOrientation(const string& filePath, const string& tfName, double& r, double& p, double& y){
    try{
        YAML::Node ori = YAML::LoadFile(filePath)[tfName]["transformation"]["orientation"];
        if (!ori){
            return false;
        }
        r = ori["r"].as<double>();
        p = ori["p"].as<double>();
        y = ori["y"].as<double>();
        return true;
    }
    catch (const exception& e){
        cerr << "WARNING! Failed to read orientation of " << tfName << ": " << e.what() << endl;
    }
    return false;
}

bool TFConfigWriter::updateTransformation(const string& filePath, const string& tfName,
                                          const vector<pair<string, string>>& values, const string& header,
                                          string& backupPath, string& errorMsg){
    ifstream fin(filePath, ios::binary);
    if (!fin){
        errorMsg = "Can not open " + filePath;
        return false;
    }
    stringstream buffer;
    buffer << fin.rdbuf();
    string content = buffer.str();
    fin.close();

    // Split in lines, keeping track of the line ending used by the file
    string eol = content.find("\r\n") != string::npos ? "\r\n" : "\n";
    vector<string> lines;
    size_t start = 0;
    while (start < content.size()){
        size_t end = content.find('\n', start);
        if (end == string::npos) end = content.size();
        string line = content.substr(start, end - start);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        lines.push_back(line);
        start = end + 1;
    }
    bool trailingEol = !content.empty() && content.back() == '\n';

    // Block of the transform: from "tfName:" to the next top level key
    int blockStart = -1;
    for (size_t i = 0; i < lines.size(); i++){
        if (lines[i].compare(0, tfName.size() + 1, tfName + ":") == 0){
            blockStart = i;
            break;
        }
    }
    if (blockStart < 0){
        errorMsg = "No '" + tfName + "' found in " + filePath;
        return false;
    }
    int blockEnd = lines.size();
    for (size_t i = blockStart + 1; i < lines.size(); i++){
        if (!isCommentOrEmpty(lines[i]) && indentOf(lines[i]) == 0){
            blockEnd = i;
            break;
        }
    }

    // "transformation:" region inside the block
    int transLine = -1;
    for (int i = blockStart + 1; i < blockEnd; i++){
        if (!isCommentOrEmpty(lines[i]) && startsWithKey(lines[i], "transformation")){
            transLine = i;
            break;
        }
    }
    if (transLine < 0){
        errorMsg = "No 'transformation' found in '" + tfName + "'";
        return false;
    }
    size_t transIndent = indentOf(lines[transLine]);
    int transEnd = blockEnd;
    for (int i = transLine + 1; i < blockEnd; i++){
        if (!isCommentOrEmpty(lines[i]) && indentOf(lines[i]) <= transIndent){
            transEnd = i;
            break;
        }
    }

    // Find the current values. They are commented out (block style values with all their children).
    set<int> toComment;
    int insertAt = -1;
    size_t keyIndent = transIndent + 2;
    bool keyFound = false;
    for (const auto& kv: values){
        for (int i = transLine + 1; i < transEnd; i++){
            if (isCommentOrEmpty(lines[i]) || !startsWithKey(lines[i], kv.first) || indentOf(lines[i]) <= transIndent){
                continue;
            }
            size_t indent = indentOf(lines[i]);
            if (!keyFound){
                keyIndent = indent;
                keyFound = true;
            }
            int j = i;
            toComment.insert(j);
            while (j + 1 < transEnd && !isCommentOrEmpty(lines[j + 1]) && indentOf(lines[j + 1]) > indent){
                toComment.insert(++j);
            }
            insertAt = max(insertAt, j + 1);
            break;
        }
    }
    if (!keyFound){
        // A transformation written as a 4x4 matrix can not be edited
        for (int i = transLine + 1; i < transEnd; i++){
            if (!isCommentOrEmpty(lines[i]) && lines[i][indentOf(lines[i])] == '-'){
                errorMsg = "'" + tfName + "' transformation is not written as position/orientation";
                return false;
            }
        }
        insertAt = transLine + 1;
    }

    string indent(keyIndent, ' ');
    vector<string> newLines;
    newLines.push_back(indent + "# " + header);
    for (const auto& kv: values){
        newLines.push_back(indent + kv.first + ": " + kv.second);
    }

    // Assemble the new file
    string out;
    for (int i = 0; i <= (int)lines.size(); i++){
        if (i == insertAt){
            if (i == (int)lines.size() && !trailingEol && !lines.empty()) out += eol;
            for (const auto& l: newLines) out += l + eol;
        }
        if (i == (int)lines.size()) break;
        string line = lines[i];
        if (toComment.count(i)){
            size_t n = min(indentOf(line), keyIndent);
            line = line.substr(0, n) + "# " + line.substr(n);
        }
        out += line;
        if (i < (int)lines.size() - 1 || trailingEol) out += eol;
    }

    // Backup the original file with a timestamp, then write the new one
    string backupBase = filePath + "." + timestamp("%Y%m%d_%H%M%S");
    backupPath = backupBase + ".bkp";
    for (int k = 1; ifstream(backupPath).good(); k++){
        backupPath = backupBase + "_" + to_string(k) + ".bkp";
    }
    ofstream fbkp(backupPath, ios::binary);
    if (!fbkp){
        errorMsg = "Can not write backup " + backupPath;
        return false;
    }
    fbkp << content;
    fbkp.close();

    ofstream fout(filePath, ios::binary);
    if (!fout){
        errorMsg = "Can not write " + filePath;
        return false;
    }
    fout << out;
    fout.close();
    return true;
}

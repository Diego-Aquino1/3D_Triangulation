//
//  OBJParser.hpp
//  Augmented Reality
//

#ifndef parseOBJ_hpp
#define parseOBJ_hpp

#include <stdio.h>

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/core.hpp>

using namespace std;

void parse_file(std::string fName, vector<cv::Point3f> &vertices,
               vector<cv::Point3f> &normals, vector<std::vector<int>> &faceVertices,
                  vector<int> &faceNormals);
std::vector<std::string> split(std::string &str, char delim);
#endif /* parseOBJ_hpp */

//
//  OBJParser.cpp
//  Augmented Reality
//

#include "parseOBJ.hpp"

using namespace std;

// Parses an obj file and gives the vertices, face vertices, normals and face normals of the object
void parse_file(std::string f_name, vector<cv::Point3f> &vertices,
            vector<cv::Point3f> &normals, vector<std::vector<int>> &face_vertices,
               vector<int> &faceNormals)
{
    string currentLine;
    ifstream inFile(f_name);
    vector<string> tokens, indices;

    if (!inFile.is_open())
    {
        cout << "Failed to open obj file: " << f_name << endl;
        return;
    }

    // Read file line by line
    while (getline(inFile, currentLine))
    {
        // Split current line into tokens
        tokens = split(currentLine, ' ');
        if (tokens.size() > 0)
        {
            vector<int> face_ind;
            // if line is a vertex, append to vertices
            if (tokens[0].compare("v") == 0)
            {
                vertices.push_back(cv::Point3f(std::stof(tokens[1]),std::stof(tokens[3]),(std::stof(tokens[2]))));
            }
            // if line is a vertex normal, append to normals
            else if (tokens[0].compare("vn") == 0)
            {
                normals.push_back(cv::Point3f(std::stof(tokens[1]),std::stof(tokens[2]),std::stof(tokens[3])));
            }
            // if line is a face vertex, append all vertex indices to face vertices
            else if (tokens[0].compare("f") == 0)
            {
                for(int i = 1 ; i<tokens.size() ; i++)
                {
                    // face vertex and normals are delimited by //
                    indices = split(tokens[i], '/');
                    face_ind.push_back(std::stoi(indices[0]));
                }
                face_vertices.push_back(face_ind);
                face_ind.clear();
            }
            
        }
    }
}

// function splits a given string at the delimiter
std::vector<std::string> split(std::string &str, char delim)
{
    std::vector<std::string> tokens;
    std::string sub_str;

    std::stringstream str_stream(str);

    while (std::getline(str_stream, sub_str, delim))
    {
        tokens.push_back(sub_str);
    }

    return tokens;
}


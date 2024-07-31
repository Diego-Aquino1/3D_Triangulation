//
//  calibrate.hpp
//  Augmented Reality
//
//  Calibration

#ifndef calibrateCamera_hpp
#define calibrateCamera_hpp

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types_c.h>

using namespace cv;
using namespace std;

void extract_draw_corners(cv::Mat& src, std::vector<std::vector<cv::Vec3f> >& point_list, std::vector<std::vector<cv::Point2f> >& corner_list);
void add_to_image_set(std::vector<cv::Point2f> corner_set, std::vector<std::vector<cv::Vec3f> >& point_list, std::vector<std::vector<cv::Point2f> >& corner_list);
void create_image_set(string path, std::vector<std::vector<cv::Vec3f> >& point_list, std::vector<std::vector<cv::Point2f> >& corner_list);
float calibrate_camera(std::vector<std::vector<cv::Vec3f>> &point_list,
                       std::vector<std::vector<cv::Point2f>> &corner_list,
                       cv::Mat &camera_matrix, cv::Mat &distortion_coeff);
bool camera_position(cv::Mat& src,cv::Mat &camera_matrix, cv::Mat &distortion_coeff,
                     cv::Mat& rotational_vec, cv::Mat& trans_vec, string objType,
                     vector<cv::Point3f> vertices = vector<cv::Point3f>(),
                     vector<std::vector<int>> face_vertices=vector<std::vector<int>>());
void draw_axes(cv::Mat& src,cv::Mat &camera_matrix, cv::Mat &distortion_coeff,
               cv::Mat& R, cv::Mat& T);

void draw_3d_obj_object(cv::Mat &src,cv::Mat &camera_matrix, cv::Mat &distortion_coeff,
                        cv::Mat& R, cv::Mat& T, vector<cv::Point3f> vertices, vector<std::vector<int>> face_vertices);
void draw_cube(cv::Mat& src, cv::Mat& camera_matrix, cv::Mat& distortion_coeff,
    cv::Mat& R, cv::Mat& T);

void harris_corner(cv::Mat& src);

#endif /* calibrate_hpp */

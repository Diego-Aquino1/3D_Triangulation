////
//  calibrateCamera.cpp
//  Calibración de Cámara y Realidad Aumentada
//

#include "calibrateCamera.hpp"
#include <filesystem>
#include <string>
#include <iostream>
#include <regex>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/features2d.hpp>

using namespace cv;
using namespace std;

// Función para detectar el tablero de ajedrez, extraer sus esquinas y dibujarlas
void extract_draw_corners(cv::Mat& src, std::vector<std::vector<cv::Vec3f>>& point_list, std::vector<std::vector<cv::Point2f>>& corner_list) {
    cv::Mat gray;
    std::vector<cv::Vec3f> point_set;
    std::vector<cv::Point2f> corner_set;
    cvtColor(src, gray, cv::COLOR_RGB2GRAY);

    Size patternsize(9, 6);
    bool patternfound = findChessboardCorners(gray, patternsize, corner_set,
        CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

    if (patternfound)
        cornerSubPix(gray, corner_set, Size(11, 11), Size(-1, -1),
            TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    
    drawChessboardCorners(src, patternsize, Mat(corner_set), patternfound);
    add_to_image_set(corner_set, point_list, corner_list);
}

// Función para añadir las esquinas del tablero de ajedrez a las listas de puntos y esquinas
void add_to_image_set(std::vector<cv::Point2f> corner_set, std::vector<std::vector<cv::Vec3f>>& point_list, std::vector<std::vector<cv::Point2f>>& corner_list) {
    std::vector<cv::Vec3f> point_set;
    Size patternsize(9, 6);

    for (int i = 0; i < patternsize.height; i++) {
        for (int j = 0; j < patternsize.width; j++) {
            point_set.push_back(cv::Point3f(j, -i, 0));
        }
    }

    point_list.push_back(point_set);
    corner_list.push_back(corner_set);
}

// Función para crear un conjunto de imágenes de calibración desde todas las imágenes guardadas en la ruta
void create_image_set(string path, std::vector<std::vector<cv::Vec3f>>& point_list, std::vector<std::vector<cv::Point2f>>& corner_list) {
    for (const auto& entry : filesystem::directory_iterator(path)) {
        string filename = entry.path().c_str();
        if (filename.substr(filename.length() - 3) == "png") {
            cout << "Procesando " << filename << endl;
            cv::Mat img = imread(filename);
            extract_draw_corners(img, point_list, corner_list);
        }
    }
}

// Función para calibrar la cámara utilizando diferentes imágenes y obtener los parámetros intrínsecos
float calibrate_camera(std::vector<std::vector<cv::Vec3f>>& point_list,
                       std::vector<std::vector<cv::Point2f>>& corner_list,
                       cv::Mat& camera_matrix, cv::Mat& distortion_coeff) {
    Size frame_size(1280, 720);
    std::vector<cv::Mat> R, T;
    float error = cv::calibrateCamera(point_list,
                                      corner_list,
                                      frame_size,
                                      camera_matrix,
                                      distortion_coeff,
                                      R,
                                      T,
                                      cv::CALIB_FIX_ASPECT_RATIO,
                                      cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, DBL_EPSILON));

    std::cout << "cameraMatrix : " << camera_matrix << std::endl;
    std::cout << "distortion Coeffs : " << distortion_coeff << std::endl;
    std::cout << "Rotation vector : " << R[0] << std::endl;
    std::cout << "Translation vector : " << T[0] << std::endl;
    cout << "Error:" << error << std::endl;
    return error;
}

// Función para determinar la posición actual de la cámara. Devuelve matrices R y T
bool camera_position(cv::Mat& src, cv::Mat& camera_matrix, cv::Mat& distortion_coeff,
                     cv::Mat& rotational_vec, cv::Mat& trans_vec, string objType, vector<cv::Point3f> vertices, vector<std::vector<int>> face_vertices) {
    cv::Mat gray;
    std::vector<cv::Vec3f> point_set;
    std::vector<cv::Point2f> corner_set;
    cvtColor(src, gray, cv::COLOR_RGB2GRAY);

    Size patternsize(9, 6);
    bool patternfound = findChessboardCorners(gray, patternsize, corner_set,
        CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

    if (patternfound)
        cornerSubPix(gray, corner_set, Size(11, 11), Size(-1, -1),
            TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

    for (int i = 0; i < patternsize.height; i++) {
        for (int j = 0; j < patternsize.width; j++) {
            point_set.push_back(cv::Point3f(j, -i, 0));
        }
    }

    if (patternfound) {
        Scalar green(0, 255, 0);
        cv::solvePnP(point_set, corner_set, camera_matrix, distortion_coeff, rotational_vec, trans_vec);
        if (objType == "Axes")
            draw_axes(src, camera_matrix, distortion_coeff, rotational_vec, trans_vec);
        else if (objType == "Obj")
            draw_3d_obj_object(src, camera_matrix, distortion_coeff, rotational_vec, trans_vec, vertices, face_vertices);
        else if (objType == "Cube")
            draw_cube(src, camera_matrix, distortion_coeff, rotational_vec, trans_vec);
    }

    return patternfound;
}

// Función para dibujar ejes 3D
void draw_axes(cv::Mat& src, cv::Mat& camera_matrix, cv::Mat& distortion_coeff,
               cv::Mat& R, cv::Mat& T) {
    vector<cv::Vec3f> real_points;
    std::vector<cv::Point2f> image_points;

    real_points.push_back(cv::Vec3f({0, 0, 0}));
    real_points.push_back(cv::Vec3f({5, 0, 0}));
    real_points.push_back(cv::Vec3f({0, -5, 0}));
    real_points.push_back(cv::Vec3f({0, 0, 5}));
    
    cv::projectPoints(real_points, R, T, camera_matrix, distortion_coeff, image_points);
    
    cv::arrowedLine(src, image_points[0], image_points[1], cv::Scalar(0, 0, 255), 3);
    cv::arrowedLine(src, image_points[0], image_points[2], cv::Scalar(0, 255, 0), 3);
    cv::arrowedLine(src, image_points[0], image_points[3], cv::Scalar(255, 0, 0), 3);
}

// Dibuja un objeto 3D a partir de archivos OBJ, utilizando vértices y caras analizadas
void draw_3d_obj_object(cv::Mat& src, cv::Mat& camera_matrix, cv::Mat& distortion_coeff,
                        cv::Mat& R, cv::Mat& T, vector<cv::Point3f> vertices, vector<std::vector<int>> face_vertices) {
    vector<cv::Point2f> image_vertices;
    Scalar color(255, 0, 255);
    int thickness = 1;
    
    cv::projectPoints(vertices, R, T, camera_matrix, distortion_coeff, image_vertices);
    
    for (int i = 0; i < face_vertices.size(); i++) {
        for (int j = 0; j < face_vertices[i].size() - 1; j++) {
            cv::line(src, image_vertices[face_vertices[i][j] - 1], image_vertices[face_vertices[i][j + 1] - 1], color, thickness);
        }
    }
}

// Cubo de ejemplo
void draw_cube(cv::Mat& src, cv::Mat& camera_matrix, cv::Mat& distortion_coeff,
    cv::Mat& R, cv::Mat& T)
{
    vector<cv::Vec3f> real_points;
    vector<cv::Point2f> image_points;
    //Scalar color(255, 0, 0);
    // cube points
    real_points.push_back(cv::Vec3f({ 5, -4, 0 }));
    real_points.push_back(cv::Vec3f({ 5, -4, 2 }));
    real_points.push_back(cv::Vec3f({ 3, -4, 0 }));
    real_points.push_back(cv::Vec3f({ 3, -4, 2 }));
    real_points.push_back(cv::Vec3f({ 5, -2, 0 }));
    real_points.push_back(cv::Vec3f({ 5, -2, 2 }));
    real_points.push_back(cv::Vec3f({ 3, -2, 0 }));
    real_points.push_back(cv::Vec3f({ 3, -2, 2 }));


    cv::projectPoints(real_points, R, T, camera_matrix, distortion_coeff, image_points);
    //Lines to connect the points
    cv::line(src, image_points[0], image_points[2], cv::Scalar(0, 0, 255),2);
    cv::line(src, image_points[4], image_points[6], cv::Scalar(0, 0, 255),2);
    cv::line(src, image_points[0], image_points[4], cv::Scalar(0, 0, 255),2);
    cv::line(src, image_points[3], image_points[7], cv::Scalar(0, 0, 255),2);
    cv::line(src, image_points[2], image_points[3], cv::Scalar(0, 0, 255),2);
    cv::line(src, image_points[4], image_points[5], cv::Scalar(0, 0, 255),2);
    cv::line(src, image_points[6], image_points[7], cv::Scalar(0, 0, 255),2);
    cv::line(src, image_points[0], image_points[1], cv::Scalar(0, 0, 255),2);
    cv::line(src, image_points[2], image_points[6], cv::Scalar(0, 0, 255),2);
    cv::line(src, image_points[1], image_points[3], cv::Scalar(0, 0, 255),2);
    cv::line(src, image_points[5], image_points[7], cv::Scalar(0, 0, 255),2);
    cv::line(src, image_points[1], image_points[5], cv::Scalar(0, 0, 255),2);
    

}

// Deteccion de esquinas
void harris_corner(cv::Mat& src) {
    cv::Mat  src_gray;
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;

    // Threshold
    int thresh = 200;

    cvtColor(src, src_gray, COLOR_BGR2GRAY);

    Mat dst = Mat::zeros(src_gray.size(), CV_32FC1);
    cornerHarris(src_gray, dst, blockSize, apertureSize, k);

    Mat dst_norm, dst_norm_scaled;
    Mat output = src.clone();
    normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
    convertScaleAbs(dst_norm, dst_norm_scaled);
    //displaying points on the image/video frame
    for (int i = 0; i < dst_norm.rows; i++)
    {
        for (int j = 0; j < dst_norm.cols; j++)
        {
            if ((int)dst_norm.at<float>(i, j) > thresh)
            {
                circle(output, Point(j, i), 2, Scalar(255, 0, 0), 2, 8, 0);
            }
        }
    }
    imshow("Harris_Corner", output);
}

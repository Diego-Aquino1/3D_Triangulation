//
//  main.cpp
//  Calibration and AugmentedReality
//

#include <iostream>
#include <string>
#include <regex>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/features2d.hpp>
#include "../calibrateCamera.hpp"
#include "../parseOBJ.hpp"

using namespace std;
using namespace cv;

int main(int argc, char* argv[]) {
    // Declaraciones
    cv::VideoCapture *capdev; // Captura de video
    cv::Mat frame, distortion_coeff, img, rotational_vec, trans_vec; // Matrices de OpenCV para la imagen y las transformaciones

    bool patternfound; // Variable para verificar si se encontró el patrón
    string calibration_image_path = "/Users/diego/Desktop/U/5/CG/Augmented Reality CG/calibration_images"; // Ruta a las imágenes de calibración
    
    string obj_path = "/Users/diego/Desktop/U/5/CG/Augmented Reality CG/Augmented Reality CG/pato3.obj"; // Ruta al archivo .obj del objeto 3D a mostrar
    
    FileStorage fs_write; // Para escribir en archivos
    FileStorage fs_read; // Para leer de archivos
    std::vector<std::vector<cv::Vec3f> > point_list; // Lista de puntos en coordenadas del mundo
    std::vector<std::vector<cv::Point2f> > corner_list; // Lista de puntos de la imagen
    
    char pressed_key = 'o'; // Tecla presionada
    int min_calibrate_images = 5; // Mínimo de imágenes para calibrar
    string save_path; // Ruta para guardar imágenes calibradas
 
    std::vector<cv::Point3f> vertices; // Vértices del objeto 3D
    std::vector<cv::Point3f> normals; // Normales del objeto 3D
    std::vector<std::vector<int>> face_vertices; // Caras del objeto 3D
    std::vector<int> face_normals; // Normales de las caras del objeto 3D
    parse_file(obj_path, vertices, normals, face_vertices, face_normals); // Parsear el archivo .obj

    capdev = new cv::VideoCapture(0); // Captura de video desde la cámara predeterminada
    capdev->set(cv::CAP_PROP_FRAME_WIDTH, 1280); // Ajustar el ancho del video
    capdev->set(cv::CAP_PROP_FRAME_HEIGHT, 720); // Ajustar la altura del video
    if (!capdev->isOpened()) {
        printf("Unable to open video device\n"); // Verificar si la cámara se abrió correctamente
        return (-1);
    }
    cv::namedWindow("Video", 1); // Crear una ventana para mostrar el video

    // Obtener algunas propiedades de la imagen
    cv::Size refS((int) capdev->get(cv::CAP_PROP_FRAME_WIDTH), (int) capdev->get(cv::CAP_PROP_FRAME_HEIGHT));
    printf("Expected size: %d %d\n", refS.width, refS.height);
    float cols = refS.width / 2;
    float rows = refS.height / 2;
    
    double mat_init[3][3] = {{1, 0, cols}, {0, 1, rows}, {0, 0, 1}};
    
    // Crear y inicializar la matriz de la cámara
    cv::Mat camera_matrix = cv::Mat(3, 3, CV_64FC1, &mat_init);
    cout << "Initialized camera matrix" << endl;
    // Archivo para guardar las propiedades de la cámara
    fs_read = FileStorage("/Users/diego/Desktop/U/5/CG/Augmented Reality CG/intrinsic.yml", FileStorage::READ);
    fs_read["camera_matrix"] >> camera_matrix;
    fs_read["distortion_coeff"] >> distortion_coeff;
    fs_read.release();
    cout << "Read camera matrix" << endl;

    for (;;) {
        *capdev >> frame; // Obtener un nuevo cuadro de la cámara, tratar como un flujo
        if (frame.empty()) {
            printf("frame is empty\n"); // Verificar si el cuadro está vacío
            break;
        }
    
        // Verificar si hay una tecla presionada
        char key = cv::waitKey(10);
        if (key != -1)
            pressed_key = key;

        switch (pressed_key) {
            case '1':
                // Guardar una imagen de calibración
                save_path = calibration_image_path + "/10.png";
                imwrite(save_path, frame);
                
                point_list.clear();
                corner_list.clear();
                create_image_set(calibration_image_path, point_list, corner_list); // Crear el conjunto de imágenes para calibrar
                if (point_list.size() > min_calibrate_images) {
                    calibrate_camera(point_list, corner_list, camera_matrix, distortion_coeff); // Calibrar la cámara
                    
                    fs_write = FileStorage("/Users/diego/Desktop/U/5/CG/Augmented Reality CG/intrinsic.yml", FileStorage::WRITE);
                    fs_write << "camera_matrix" << camera_matrix;
                    fs_write << "distortion_coeff" << distortion_coeff;
                    fs_write.release();
                } else {
                    cout << "Pocas imagenes para calibrar";
                }
                pressed_key = 'o';
                cv::imshow("Video", frame); // Mostrar el cuadro de video
                break;
                
            case '2':
                harris_corner(frame); // Detectar esquinas con Harris
                cv::imshow("Video", frame); // Mostrar el cuadro de video
                break;
                
            default:
                cv::imshow("Video", frame); // Mostrar el cuadro de video
                break;

            case '3':
                patternfound = camera_position(frame, camera_matrix, distortion_coeff, rotational_vec, trans_vec, "Axes"); // Determinar la posición de la cámara
                if (patternfound) {
                    // Imprimir las matrices de rotación y traducción en tiempo real
                    std::cout << "Rotation Matrix: " << std::endl;
                    for (int i = 0; i < rotational_vec.rows; i++) {
                        for (int j = 0; j < rotational_vec.cols; j++) {
                            std::cout << rotational_vec.at<cv::Vec2f>(i, j) << std::endl;
                        }
                    }
                    
                    std::cout << "Translation Matrix: " << std::endl;
                    for (int i = 0; i < trans_vec.rows; i++) {
                        for (int j = 0; j < trans_vec.cols; j++) {
                            std::cout << trans_vec.at<cv::Vec2f>(i, j) << std::endl;
                        }
                    }
                }
                cv::imshow("Video", frame); // Mostrar el cuadro de video
                break;
                
            case '4':
                camera_position(frame, camera_matrix, distortion_coeff, rotational_vec, trans_vec, "Cube"); // Determinar la posición de la cámara y mostrar un cubo
                cv::imshow("Video", frame); // Mostrar el cuadro de video
                break;

            case '5':
                camera_position(frame, camera_matrix, distortion_coeff, rotational_vec, trans_vec, "Obj", vertices, face_vertices); // Determinar la posición de la cámara y mostrar un objeto 3D
                cv::imshow("Video", frame); // Mostrar el cuadro de video
                break;
        }
        // Salir si se presiona la tecla 'q'
        if (key == 'q')
            break;
    }

    return (0);
}

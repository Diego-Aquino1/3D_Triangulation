#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>
#include <map>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "Shader.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_3<K> Delaunay;
typedef K::Point_3 Point_3;

void loadOBJ(const std::string& path, std::vector<Point_3>& points) {
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs);

    if (!scene || !scene->mMeshes) {
        std::cerr << "ERROR::ASSIMP::" << importer.GetErrorString() << std::endl;
        return;
    }

    for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
        aiMesh* mesh = scene->mMeshes[i];
        for (unsigned int j = 0; j < mesh->mNumVertices; j++) {
            aiVector3D vertex = mesh->mVertices[j];
            points.push_back(Point_3(vertex.x, vertex.y, vertex.z));
        }
    }
}

void triangulateAndRender(const std::vector<Point_3>& points, Shader& shader, const glm::mat4& view, const glm::mat4& projection) {
    Delaunay dt;
    dt.insert(points.begin(), points.end());

    std::vector<float> vertexData;
    std::vector<unsigned int> indexData;
    std::map<Point_3, unsigned int> vertexIndexMap;

    // Asignar índices a los vértices
    unsigned int index = 0;
    for (auto v = dt.finite_vertices_begin(); v != dt.finite_vertices_end(); ++v) {
        const Point_3& point = v->point();
        vertexData.push_back(point.x());
        vertexData.push_back(point.y());
        vertexData.push_back(point.z());
        vertexIndexMap[point] = index++;
    }

    // Obtener las aristas de los tetraedros en lugar de las caras completas
    std::set<std::pair<unsigned int, unsigned int>> edges;
    for (auto cell = dt.finite_cells_begin(); cell != dt.finite_cells_end(); ++cell) {
        for (int i = 0; i < 4; ++i) {
            unsigned int indices[3];
            for (int j = 0; j < 3; ++j) {
                indices[j] = vertexIndexMap[cell->vertex((i + j) % 4)->point()];
            }
            edges.insert({std::min(indices[0], indices[1]), std::max(indices[0], indices[1])});
            edges.insert({std::min(indices[1], indices[2]), std::max(indices[1], indices[2])});
            edges.insert({std::min(indices[2], indices[0]), std::max(indices[2], indices[0])});
        }
    }

    // Convertir el conjunto de aristas a un índice de datos
    for (const auto& edge : edges) {
        indexData.push_back(edge.first);
        indexData.push_back(edge.second);
    }

    GLuint VAO, VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(float), vertexData.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexData.size() * sizeof(unsigned int), indexData.data(), GL_STATIC_DRAW);

    // Configuración de los atributos de los vértices
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    // Configuración del shader
    shader.Use();
    glm::mat4 model = glm::mat4(1.0f);
    GLuint modelLoc = glGetUniformLocation(shader.Program, "model");
    GLuint viewLoc = glGetUniformLocation(shader.Program, "view");
    GLuint projectionLoc = glGetUniformLocation(shader.Program, "projection");
    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection));

    // Dibujar líneas en lugar de triángulos
    glBindVertexArray(VAO);
    glDrawElements(GL_LINES, indexData.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
}

int main() {
    // Inicialización de GLFW y OpenGL
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(800, 600, "OpenGL Delaunay Triangulation", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        return -1;
    }

    glViewport(0, 0, 800, 600);

    Shader shader;

    std::vector<Point_3> points;
    loadOBJ("/Users/diego/Downloads/single_pumpkin.obj", points);

    // Inicialización de OpenCV y detección de patrón
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera" << std::endl;
        return -1;
    }

    cv::Mat frame;
    cv::Mat cameraMatrix, distCoeffs;

    // Calibrar cámara o cargar parámetros pre-calibrados
    // Aquí asumimos que ya tienes los parámetros de la cámara
    // cv::FileStorage fs("calibration_data.yml", cv::FileStorage::READ);
    // fs["camera_matrix"] >> cameraMatrix;
    // fs["distortion_coefficients"] >> distCoeffs;

    std::vector<cv::Point2f> imagePoints;
    std::vector<cv::Point3f> objectPoints;
    // Definir puntos del patrón de ajedrez
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 6; j++) {
            objectPoints.emplace_back(j * 0.025f, i * 0.025f, 0.0f);
        }
    }

    while (!glfwWindowShouldClose(window)) {
        cap >> frame;
        if (frame.empty()) break;

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        bool found = cv::findChessboardCorners(gray, cv::Size(6, 9), imagePoints);

        if (found) {
            cv::Mat rvec, tvec;
            cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

            cv::Mat rotMat;
            cv::Rodrigues(rvec, rotMat);

            glm::mat4 view = glm::mat4(1.0f);
            // Copiar la matriz de rotación desde OpenCV a glm
            view[0][0] = rotMat.at<double>(0, 0);
            view[0][1] = rotMat.at<double>(0, 1);
            view[0][2] = rotMat.at<double>(0, 2);
            view[1][0] = rotMat.at<double>(1, 0);
            view[1][1] = rotMat.at<double>(1, 1);
            view[1][2] = rotMat.at<double>(1, 2);
            view[2][0] = rotMat.at<double>(2, 0);
            view[2][1] = rotMat.at<double>(2, 1);
            view[2][2] = rotMat.at<double>(2, 2);
            view[3][0] = tvec.at<double>(0);
            view[3][1] = tvec.at<double>(1);
            view[3][2] = tvec.at<double>(2);

            glm::mat4 projection = glm::perspective(glm::radians(45.0f), 4.0f / 3.0f, 0.1f, 100.0f);

            triangulateAndRender(points, shader, view, projection);
        }

        cv::imshow("Camera", frame);
        if (cv::waitKey(30) >= 0) break;

        glfwPollEvents();
        glfwSwapBuffers(window);
    }

    glfwTerminate();
    return 0;
}

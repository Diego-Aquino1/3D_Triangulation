#include <GL/glew.h>
#include <GLFW/glfw3.h>
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

/*
void triangulateAndRender(const std::vector<Point_3>& points, Shader& shader) {
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
        // Color por defecto (naranja)
        vertexData.push_back(1.0f); // Rojo
        vertexData.push_back(0.5f); // Verde
        vertexData.push_back(0.2f); // Azul
        vertexIndexMap[point] = index++;
    }

    // Obtener las caras (caras de tetraedros en 3D)
    for (auto cell = dt.finite_cells_begin(); cell != dt.finite_cells_end(); ++cell) {
        for (int i = 0; i < 4; ++i) {
            unsigned int indices[3];
            for (int j = 0; j < 3; ++j) {
                indices[j] = vertexIndexMap[cell->vertex((i + j) % 4)->point()];
            }

            // Usar colores distintos para cada triángulo
            glm::vec3 color1 = glm::vec3(1.0f, 0.0f, 0.0f); // Rojo
            glm::vec3 color2 = glm::vec3(0.0f, 1.0f, 0.0f); // Verde
            glm::vec3 color3 = glm::vec3(0.0f, 0.0f, 1.0f); // Azul

            // Añadir colores para cada triángulo
            for (int j = 0; j < 3; ++j) {
                vertexData.push_back((j == 0) ? color1.r : (j == 1) ? color2.r : color3.r);
                vertexData.push_back((j == 0) ? color1.g : (j == 1) ? color2.g : color3.g);
                vertexData.push_back((j == 0) ? color1.b : (j == 1) ? color2.b : color3.b);
            }

            // Triángulos de la cara del tetraedro
            indexData.push_back(indices[0]);
            indexData.push_back(indices[1]);
            indexData.push_back(indices[2]);
        }
    }
 



    std::cout << "Vertex data size: " << vertexData.size() << std::endl;
    std::cout << "Index data size: " << indexData.size() << std::endl;

    GLuint VAO, VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(float), vertexData.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexData.size() * sizeof(unsigned int), indexData.data(), GL_STATIC_DRAW);

    // Configuración de los atributos de los vértices (posición + color)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0); // Posición
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float))); // Color
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    // Configuración del shader
    shader.Use();
    glm::mat4 model = glm::mat4(1.0f);
    glm::mat4 view = glm::lookAt(glm::vec3(0.0f, 0.0f, 3.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    glm::mat4 projection = glm::perspective(glm::radians(45.0f), 800.0f / 600.0f, 0.1f, 100.0f);
    GLuint modelLoc = glGetUniformLocation(shader.Program, "model");
    GLuint viewLoc = glGetUniformLocation(shader.Program, "view");
    GLuint projectionLoc = glGetUniformLocation(shader.Program, "projection");
    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection));

    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, indexData.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
}
 
 */

void triangulateAndRender(const std::vector<Point_3>& points, Shader& shader) {
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

    std::cout << "Vertex data size: " << vertexData.size() << std::endl;
    std::cout << "Index data size: " << indexData.size() << std::endl;

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
    glm::mat4 view = glm::lookAt(glm::vec3(0.0f, 0.0f, 3.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    glm::mat4 projection = glm::perspective(glm::radians(45.0f), 800.0f / 600.0f, 0.1f, 100.0f);
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
    // loadOBJ("/Users/diego/Downloads/single_pumpkin.obj", points);
    loadOBJ("/Users/diego/Downloads/PatoGozu/patitoGozu.obj", points);

    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        triangulateAndRender(points, shader);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}

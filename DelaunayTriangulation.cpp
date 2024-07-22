#include "DelaunayTriangulation.hpp"

DelaunayTriangulation::DelaunayTriangulation() {}

void DelaunayTriangulation::generatePoints() {
    points = { Point(0, 0, 0), Point(1, 0, 0), Point(0, 1, 0), Point(0, 0, 1) };
}

void DelaunayTriangulation::triangulate() {
    dt.insert(points.begin(), points.end());
}

void DelaunayTriangulation::createBuffers(GLuint &VAO, GLuint &numIndices) {
    std::vector<GLfloat> vertices;
    std::vector<GLuint> indices;
    std::map<Point, GLuint> pointIndexMap;
    GLuint index = 0;

    for (auto v = dt.finite_vertices_begin(); v != dt.finite_vertices_end(); ++v) {
        const Point& p = v->point();
        vertices.push_back(p.x());
        vertices.push_back(p.y());
        vertices.push_back(p.z());
        pointIndexMap[p] = index++;
    }

    for (auto c = dt.finite_cells_begin(); c != dt.finite_cells_end(); ++c) {
        for (int i = 0; i < 4; ++i) {
            std::vector<GLuint> facet;
            for (int j = 1; j < 4; ++j) {
                facet.push_back(pointIndexMap[c->vertex((i + j) % 4)->point()]);
            }
            indices.insert(indices.end(), facet.begin(), facet.end());
        }
    }

    GLuint VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GLfloat), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)0);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);

    numIndices = indices.size();
}


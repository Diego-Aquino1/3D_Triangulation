#ifndef DELAUNAYTRIANGULATION_H
#define DELAUNAYTRIANGULATION_H

#include <vector>
#include <GL/glew.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_3<K> Delaunay;
typedef K::Point_3 Point;

class DelaunayTriangulation {
public:
    DelaunayTriangulation();
    void generatePoints();
    void triangulate();
    void createBuffers(GLuint &VAO, GLuint &numIndices);

    std::vector<Point> points;
    Delaunay dt;
};

#endif


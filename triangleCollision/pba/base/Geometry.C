//
// Created by jingcoz on 9/18/17.
//

# include "Geometry.h"
# include <iostream>

using namespace pba;
using namespace std;


void TriangleGeometry::add_triangle(TrianglePtr tri)
{
    triangles.push_back(tri);
}

void TriangleGeometry::cleanTrianglesCollisionStatus()
{
    for (auto it = triangles.cbegin(); it != triangles.cend(); ++it)
    {
        TrianglePtr tri = *it;
        tri->setCollisionStatus(false);
    }
}

void TriangleGeometry::build_triangles(std::vector<Vector> &vertices, std::vector<Vector> &face_indices)
{
    if (!triangles.empty()) {triangles.clear();}
    for (auto it = face_indices.cbegin(); it != face_indices.cend(); ++it)
    {
        Vector index = *it;
        Vector P0 = vertices.at(size_t(index.X()));
        Vector P1 = vertices.at(size_t(index.Y()));
        Vector P2 = vertices.at(size_t(index.Z()));
        TrianglePtr tri = new Triangle(P0, P1, P2);
        triangles.push_back(tri);
    }
}

void TriangleGeometry::build_trianglesTree(int depth)
{
    string nam = "kdTree_" + name;
    trianglesTree = new KdTree(nam, depth);
    trianglesTree->buildTree(triangles, *bbox);
}

//
// Created by jingcoz on 9/18/17.
//

# include "Geometry.h"
# include <iostream>

using namespace pba;
using namespace std;

Triangle::Triangle(const pba::Vector& p0, const pba::Vector& p1, const pba::Vector& p2): P0(p0), P1(p1), P2(p2)
{
    e1 = P1 - P0;
    e2 = P2 - P0;
    norm = (e2^e1).unitvector();
}


void TriangleGeometry::add_triangle(TrianglePtr tri)
{
    triangles.push_back(tri);
}

void TriangleGeometry::add_collisions(size_t p)
{
    collision_indices.push_back(p);
}

void TriangleGeometry::clean_collisions()
{
    collision_indices.clear();
}

void TriangleGeometry::gen_triangles(std::vector<Vector>& vertices, std::vector<Vector>& face_indices)
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

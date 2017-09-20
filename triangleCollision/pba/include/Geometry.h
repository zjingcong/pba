//
// Created by jingcoz on 9/18/17.
//

#ifndef PBA_GEOMETRY_H
#define PBA_GEOMETRY_H

# include <vector>
# include <string>
# include <memory>
# include "Vector.h"
# include "Color.h"

namespace pba
{
    class Triangle
    {
    public:
        Triangle(const pba::Vector& P0, const pba::Vector& P1, const pba::Vector& P2);
        ~Triangle() {}

        const pba::Vector& getP0() const { return P0;}
        const pba::Vector& getP1() const { return P1;}
        const pba::Vector& getP2() const { return P2;}
        const pba::Vector& getE1() const { return e1;}
        const pba::Vector& getE2() const { return e2;}
        const pba::Vector& getNorm() const { return norm;}

    private:
        pba::Vector P0;
        pba::Vector P1;
        pba::Vector P2;
        pba::Vector e1;
        pba::Vector e2;
        pba::Vector norm;
    };

    typedef Triangle* TrianglePtr;


    class TriangleGeometry
    {
    public:
        TriangleGeometry(const std::string& nam): name(nam)  {}
        ~TriangleGeometry() {}

        void add_triangle(TrianglePtr tri);
        void add_face_color(const Color& c);

        void gen_triangles(std::vector<Vector>& vertices, std::vector<Vector>& face_indices);  // here face_indices start with 0

        std::vector<TrianglePtr>& get_triangles()    { return triangles;}   // return ref!
        std::vector<Color>& get_face_colors()    { return face_colors;}
        size_t get_nb() const { return triangles.size();}
        const std::string& Name() const { return name;}

    private:
        std::string name;
        std::vector<Color> face_colors;
        std::vector<TrianglePtr> triangles;

        TriangleGeometry()  {}
    };

    typedef TriangleGeometry* GeometryPtr;
};

#endif //PBA_GEOMETRY_H

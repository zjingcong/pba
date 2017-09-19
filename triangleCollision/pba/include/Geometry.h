//
// Created by jingcoz on 9/18/17.
//

#ifndef PBA_TRIANGLE_H
#define PBA_TRIANGLE_H

# include <vector>
# include "Vector.h"

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
    typedef std::vector<TrianglePtr> Geom;
};

#endif //PBA_TRIANGLE_H

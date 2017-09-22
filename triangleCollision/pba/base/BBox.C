//
// Created by jingcoz on 9/21/17.
//

# include "BBox.h"

using namespace pba;

AABB::AABB(Vector &llc, Vector &urc)
{
    assert((llc.X() < urc.X()) && (llc.Y() < urc.Y()) && (llc.Z() < urc.Z()));  // mix < max
    LLC = llc;
    URC = urc;
}

AABB AABB::subDivide(const int i, const int id)
{
    assert(i <= 2);
    assert(id <= 1);
    Vector llc;
    Vector urc;
    switch (i)
    {
        // subdivide X
        case 0:
        {
            if (id == 0) {llc = LLC; urc = Vector(URC.X() * 0.5, URC.Y(), URC.Z());}
            else    {llc = Vector(URC.X() * 0.5, LLC.Y(), LLC.Z()); urc = URC;}
            break;
        }

        // subdivide Y
        case 1:
        {
            if (id == 0) {llc = LLC; urc = Vector(URC.X(), URC.Y() * 0.5, URC.Z());}
            else    {llc = Vector(LLC.X(), URC.Y() * 0.5, LLC.Z()); urc = URC;}
            break;
        }

        // subdivide Z
        case 2:
        {
            if (id == 0) {llc = LLC; urc = Vector(URC.X(), URC.Y(), URC.Z() * 0.5);}
            else    {llc = Vector(LLC.X(), LLC.Y(), URC.Z() * 0.5); urc = URC;}
            break;
        }

        default:
            break;
    }

    return AABB(llc, urc);
}

bool AABB::insidePoint(const Vector &P)
{
    return (lessVector(P, URC) && lessVector(LLC, P));
}

bool AABB::insideTriangle(TrianglePtr triangle)
{
    return (insidePoint(triangle->getP0()) && insidePoint(triangle->getP1()) && insidePoint(triangle->getP2()));
}

bool AABB::intersect() const
{
    return true;
}

bool AABB::lessVector(const Vector &v1, const Vector &v2)
{
    if ((v1.X() <= v2.X()) && (v1.Y() <= v2.Y()) && (v1.Z() <= v2.Z()))
    { return true;}
    else    { return false;}
}

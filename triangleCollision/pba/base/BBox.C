//
// Created by jingcoz on 9/21/17.
//

# include "BBox.h"

using namespace pba;

AABB::AABB(Vector &llc, Vector &urc)
{
    assert(lessVector(llc, urc));  // mix < max
    LLC = llc;
    URC = urc;
    center = (LLC + URC) * 0.5;
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

bool AABB::insidePoint(const Vector &P) const
{
    return (lessVector(P, URC) && lessVector(LLC, P));
}

int AABB::insideTriangle(TrianglePtr triangle) const
{
    bool b0 = insidePoint(triangle->getP0());
    bool b1 = insidePoint(triangle->getP1());
    bool b2 = insidePoint(triangle->getP2());

    if (b0 && b1 && b2) { return 1;}    // inside
    else if (b0 || b1 || b2)    { return 2;}    // across
    else    { return 0;}    // outside
}

bool AABB::intersect(const Vector& origin, const Vector& dir, const float& t0, const float& t1) const
{
    float tmin, tmax, tymin, tymax, tzmin, tzmax;
    if (dir.X() >= 0.0)
    {
        tmin = float((LLC.X() - origin.X()) / dir.X());
        tmax = float((URC.X() - origin.X()) / dir.X());
    }
    else
    {
        tmin = float((URC.X() - origin.X()) / dir.X());
        tmax = float((LLC.X() - origin.X()) / dir.X());
    }
    if (dir.Y() >= 0.0)
    {
        tymin = float((LLC.Y() - origin.Y()) / dir.Y());
        tymax = float((URC.Y() - origin.Y()) / dir.Y());
    }
    else
    {
        tymin = float((URC.Y() - origin.Y()) / dir.Y());
        tymax = float((LLC.Y() - origin.Y()) / dir.Y());
    }
    if ((tmin > tymax) || (tymin > tmax)) { return false;}
    if (tymin > tmin)   {tmin = tymin;}
    if (tymax < tmax)   {tmax = tymax;}
    if (dir.Z() >= 0.0)
    {
        tzmin = float((LLC.Z() - origin.Z()) / dir.Z());
        tzmax = float((URC.Z() - origin.Z()) / dir.Z());
    }
    else
    {
        tzmin = float((URC.Z() - origin.Z()) / dir.Z());
        tzmax = float((LLC.Z() - origin.Z()) / dir.Z());
    }
    if ((tmin > tzmax) || (tzmin < tmax))   { return false;}
    if (tzmin > tmin)   {tmin = tzmin;}
    if (tzmax < tmax)   {tmax = tzmax;}

    return ((tmin < t1) && (tmax > t0));
}

bool AABB::lessVector(const Vector &v1, const Vector &v2) const
{
    if ((v1.X() <= v2.X()) && (v1.Y() <= v2.Y()) && (v1.Z() <= v2.Z()))
    { return true;}
    else    { return false;}
}

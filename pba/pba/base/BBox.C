//
// Created by jingcoz on 9/21/17.
//

# include "BBox.h"
# include <iostream>

using namespace pba;


AABB::AABB()
{
    Vector llc(0.0, 0.0, 0.0);
    Vector urc(0.0, 0.0, 0.0);

    AABB(llc, urc);
}

AABB::AABB(Vector &llc, Vector &urc)
{
    assert(lessVector(llc, urc));  // mix < max
    LLC = llc;
    URC = urc;
    center = (LLC + URC) * 0.5;
    vec_length = URC - LLC;
}

AABB AABB::subDivide(const int i, const int id)
{
    assert(i <= 2 && i >= 0);
    assert(id <= 1 && id >= 0);
    Vector llc;
    Vector urc;
    switch (i)
    {
        // subdivide X
        case 0:
        {
            if (id == 0) {llc = LLC; urc = Vector((URC.X() + LLC.X()) * 0.5, URC.Y(), URC.Z());}
            else    {llc = Vector((URC.X() + LLC.X()) * 0.5, LLC.Y(), LLC.Z()); urc = URC;}
            break;
        }

        // subdivide Y
        case 1:
        {
            if (id == 0) {llc = LLC; urc = Vector(URC.X(), (URC.Y() + LLC.Y()) * 0.5, URC.Z());}
            else    {llc = Vector(LLC.X(), (URC.Y() + LLC.Y()) * 0.5, LLC.Z()); urc = URC;}
            break;
        }

        // subdivide Z
        case 2:
        {
            if (id == 0) {llc = LLC; urc = Vector(URC.X(), URC.Y(), (URC.Z() + LLC.Z()) * 0.5);}
            else    {llc = Vector(LLC.X(), LLC.Y(), (URC.Z() + LLC.Z()) * 0.5); urc = URC;}
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
    else    {return 0;}
}

//! AABB intersection: 0 - no intersection, 1 - intersect, 2 - inside, 3 - tangent
int AABB::intersect(const Vector& origin, const Vector& target) const
{
    if (insidePoint(origin) && insidePoint(target)) { return 2;}

    Vector dir = (target - origin).unitvector();
    float t0 = 0.0;
    float t1 = float((target - origin).magnitude());
    float tmin, tmax, tymin, tymax, tzmin, tzmax;

    float divx = float(1.0 / dir.X());
    if (divx >= 0.0)
    {
        tmin = float((LLC.X() - origin.X()) * divx);
        tmax = float((URC.X() - origin.X()) * divx);
    }
    else
    {
        tmin = float((URC.X() - origin.X()) * divx);
        tmax = float((LLC.X() - origin.X()) * divx);
    }

    float divy = float(1.0 / dir.Y());
    if (divy >= 0.0)
    {
        tymin = float((LLC.Y() - origin.Y()) * divy);
        tymax = float((URC.Y() - origin.Y()) * divy);
    }
    else
    {
        tymin = float((URC.Y() - origin.Y()) * divy);
        tymax = float((LLC.Y() - origin.Y()) * divy);
    }
    if ((tmin > tymax) || (tymin > tmax)) { return 0;}
    if (tymin > tmin)   {tmin = tymin;}
    if (tymax < tmax)   {tmax = tymax;}

    float divz = float(1.0 / dir.Z());
    if (divz >= 0.0)
    {
        tzmin = float((LLC.Z() - origin.Z()) * divz);
        tzmax = float((URC.Z() - origin.Z()) * divz);
    }
    else
    {
        tzmin = float((URC.Z() - origin.Z()) * divz);
        tzmax = float((LLC.Z() - origin.Z()) * divz);
    }
    if ((tmin > tzmax) || (tzmin > tmax))   { return 0;}
    if (tzmin > tmin)   {tmin = tzmin;}
    if (tzmax < tmax)   {tmax = tzmax;}

    if ((dir.X() == 0.0 && ((LLC.X() - origin.X()) == 0.0 || (URC.X() - origin.X()) == 0.0)) ||
        (dir.Y() == 0.0 && ((LLC.Y() - origin.Y()) == 0.0 || (URC.Y() - origin.Y()) == 0.0)) ||
        (dir.X() == 0.0 && ((LLC.Z() - origin.Z()) == 0.0 || (URC.Z() - origin.Z()) == 0.0)))   { return 3;}

    if ((tmin < t1) && (tmax > t0)) { return 1;}
    else    { return 0;}
}

bool AABB::lessVector(const Vector &v1, const Vector &v2) const
{
    if ((v1.X() <= v2.X()) && (v1.Y() <= v2.Y()) && (v1.Z() <= v2.Z()))
    { return true;}
    else    { return false;}
}

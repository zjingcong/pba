//
// Created by jingcoz on 9/13/17.
//

# include "Force.h"

using namespace pba;
using namespace std;

const Vector Gravity::getForce(const size_t& p)
{
    Vector y_unit(0, 1, 0);
    float mass = DS->mass(p);
    force = -1 * Gravity::floatParms.at("g") * mass * y_unit;

    return force;
}


const Vector BoidInnerForce::getForce(const size_t& p)
{
    Vector total_accel = boid->get_total_accel(p);
    force = total_accel * DS->mass(p);

    return force;
}


const Vector Spring::getForce(const size_t& p)
{
    float k = Spring::floatParms.at("k");
    Vector x0 = Spring::vectorParms.at("x0");
    Vector x = DS->pos(p);
    force = -k * (x - x0);

    return force;
}


const Vector MagneticForce::getForce(const size_t& p)
{
    Vector x = DS->pos(p);
    Vector xm = MagneticForce::vectorParms.at("xm");
    Vector B = MagneticForce::floatParms.at("B") * (x - xm) / pow((x - xm).magnitude(), 3.0);
    Vector v = DS->vel(p);
    force = v ^ B;

    return force;
}


//! --------------------------- Create Force Share Ptrs ------------------------------------

pba::ForcePtr pba::CreateGravity(DynamicalState ds, const float &gconstant)
{
    return ForcePtr(new Gravity(ds, gconstant));
}

pba::ForcePtr pba::CreateBoidInnerForce(DynamicalState ds, BoidPtr &b)
{
    return ForcePtr(new BoidInnerForce(ds, b));
}

pba::ForcePtr pba::CreateSpring(DynamicalState ds, const Vector &x0, const float &kconstant)
{
    return ForcePtr(new Spring(ds, x0, kconstant));
}

pba::ForcePtr pba::CreateMagneticForce(DynamicalState ds, const Vector &xm, const float &b)
{
    return ForcePtr(new MagneticForce(ds, xm, b));
}

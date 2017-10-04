//
// Created by jingcoz on 9/13/17.
//

# include "Force.h"

using namespace pba;
using namespace std;

const Vector Gravity::getForce(DynamicalState DS, const size_t p)
{
    Vector y_unit(0, 1, 0);
    float mass = DS->mass(p);
    force = -1 * Gravity::floatParms.at("g") * mass * y_unit;

    return force;
}


const Vector BoidInnerForce::getForce(DynamicalState DS, const size_t p)
{
    Vector total_accel = boid->get_total_accel(p);
    force = total_accel * DS->mass(p);

    return force;
}


const Vector Spring::getForce(DynamicalState DS, const size_t p)
{
    float k = Spring::floatParms.at("k");
    Vector x0 = Spring::vectorParms.at("x0");
    Vector x = DS->pos(p);
    force = -k * (x - x0);

    return force;
}


const Vector MagneticForce::getForce(DynamicalState DS, const size_t p)
{
    Vector x = DS->pos(p);
    Vector xm = MagneticForce::vectorParms.at("xm");
    Vector B = MagneticForce::floatParms.at("B") * (x - xm) / pow((x - xm).magnitude(), 3.0);
    Vector v = DS->vel(p);
    force = v ^ B;

    return force;
}

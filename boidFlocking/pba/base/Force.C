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
    force = -1 * Gravity::parms.at("g") * mass * y_unit;

    return force;
}

const Vector BoidInnerForce::getForce(DynamicalState DS, const size_t p)
{
    Vector total_accel = boid->get_total_accel(p);
    force = total_accel * DS->mass(p);

    return force;
}

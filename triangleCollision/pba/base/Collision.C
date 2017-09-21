//
// Created by jingcoz on 9/18/17.
//

# include <iostream>
# include "Collision.h"

using namespace std;
using namespace pba;


bool TriangleCollision::collisionDetection(const double& dt, DynamicalState DS, const size_t p, TrianglePtr triangle, double& dt_i, Vector& x_i)
{
    // detect collision with the plane
    double f0 = (DS->pos(p) - triangle->getP0()) * triangle->getNorm();
    if (f0 == 0.0)  { return false;}
    double f1 = (DS->pos(p) - DS->vel(p) * dt - triangle->getP0()) * triangle->getNorm();
    if (f0 * f1 > 0.0)  { return false;}
    // calculate dt_i and x_i
    double dti = ((DS->pos(p) - triangle->getP0()) * triangle->getNorm()) / (DS->vel(p) * triangle->getNorm());
    Vector xi = DS->pos(p) - DS->vel(p) * dti;

    // detect collision in the triangle
    Vector e1 = triangle->getE1();
    Vector e2 = triangle->getE2();

    double a = (e2^e1) * (e2^(xi - triangle->getP0())) / pow((e2^e1).magnitude(), 2.0);
    if (a < 0.0 || a > 1.0) { return false;}
    double b = (e1^e2) * (e1^(xi - triangle->getP0())) / pow((e1^e2).magnitude(), 2.0);
    if (b < 0.0 || b > 1.0) { return false;}
    double c = a + b;
    if (c < 0.0 || c > 1.0) { return false;}

    if ((dt * dti) < 0.0)    { return false;}
    if (fabs(dti) > fabs(dt))  { return false;}
    if (((dt - dti) / dt) < pow(10.0, -6.0)) { return false;}

    // parse interaction position and time
    dt_i = dti;
    x_i = xi;

    return true;
}


void TriangleCollision::collisionHandling(const double& dt, DynamicalState DS, const size_t p, TrianglePtr collision_triangle, const double& dt_i, const Vector& x_i, const double& Cr, const double& Cs)
{
    // calculate vel_r and new pos
    Vector vel = DS->vel(p);
    Vector norm = collision_triangle->getNorm();
    Vector tmp = norm * (norm * vel);
    Vector vel_perp = vel - tmp;
    Vector vel_r = Cs * vel_perp - Cr * tmp;

    // update dynamical state
    Vector pos_new = x_i + vel_r * dt_i;
    DS->set_pos(p, pos_new);
    DS->set_vel(p, vel_r);
}

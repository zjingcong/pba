//
// Created by jingcoz on 9/18/17.
//

# include "Collision.h"

using namespace std;
using namespace pba;


void TriangleCollision::collision(const double& dt, const size_t p, const double& Cr, const double& Cs)
{
    // loop until no collisions
    bool collision_flag = true;
    while(collision_flag)
    {
        // loop over triangles
        int collision_num = 0;
        for (auto it = geom.cbegin(); it != geom.cend(); ++it)
        {
            // collision detection
            TrianglePtr triangle = *it;
            if (collisionDetection(dt, p, triangle))   {collision_num++;}
            if (collision_num == 0)   {collision_flag = false;} // no collision
        }
        // pick the maximum dt_i and handle collision for maximum dt_i
        if (collision_flag) {collisionHandling(dt, p, Cr, Cs);}
    }
}


bool TriangleCollision::collisionDetection(const double& dt, const size_t p, TrianglePtr triangle)
{
    // calculate dt_i and x_i
    double dti = ((DS->pos(p) - triangle->getP0()) * triangle->getNorm()) / (DS->vel(p) * triangle->getNorm());
    x_i = DS->pos(p) - DS->vel(p) * dti;

    // collision detection
    Vector e1 = triangle->getE1();
    Vector e2 = triangle->getE2();
    double a = (e2^e1) * (e2^(x_i - triangle->getP0())) / pow((e2^e1).magnitude(), 2.0);
    if (a < 0 || a > 1) {return false;}
    double b = (e1^e2) * (e1^(x_i - triangle->getP0())) / pow((e1^e2).magnitude(), 2.0);
    if (b < 0 || b > 1) {return false;}
    double c = a + b;
    if (c < 0 || c > 1) {return false;}

    if ((dt * dti) < 0)    {return false;}
    if (fabs(dti) > fabs(dt))  {return false;}
    if ((dt - dti) / dt < pow(10.0, -6.0)) {return false;}

    // pick the maximum dt_i, and set the collision norm for handling calculation
    if (dti > dt_i)
    {
        dt_i = dti;
        collision_triangle = triangle;
    }

    return true;
}


void TriangleCollision::collisionHandling(const double& dt, const size_t p, const double& Cr, const double& Cs)
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

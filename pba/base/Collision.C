//
// Created by jingcoz on 9/18/17.
//

# include <iostream>
# include "Collision.h"

using namespace std;
using namespace pba;

void ParticleCollision::collision(const double &dt)
{
    // loop over particles
    size_t i;
    for (i = 0; i < DS->nb(); ++i)
    {
        // loop until no collisions
        bool collision_flag = true;
        double tmp_dt = dt;
        while (collision_flag)
        {
            CollisionData CD;
            collisionWithinTriangles(tmp_dt, i, geom->get_triangles(), CD);
            collision_flag = CD.collision_status;
            if (collision_flag)
            {
                // handle collision for maximum dt_i
                collisionHandling(i, CD);
                // store collision triangle
                CD.triangle->setCollisionStatus(true);
                // update dt (use maximum dti as new dt for next collision detection)
                tmp_dt = CD.dt_i;
            }
        }
    }
}

void ParticleCollision::collisionWithKdTree(const double &dt)
{
    // loop over particles
    for (size_t i = 0; i < DS->nb(); ++i)
    {
        bool collision_flag = true;
        double tmp_dt = dt;
        while (collision_flag)
        {
            // search triangles
            Vector vec1 = DS->pos(i);
            Vector vec0 = vec1 - DS->vel(i) * tmp_dt;
            CollisionData CD = geom->getKdTree()->searchCollision(shared_from_this(), tmp_dt, DS, i, vec0, vec1);
            collision_flag = CD.collision_status;

            if (collision_flag)
            {
                // handle collision for maximum dt_i
                collisionHandling(i, CD);
                // store collision triangle
                CD.triangle->setCollisionStatus(true);
                // update dt (use maximum dti as new dt for next collision detection)
                tmp_dt = CD.dt_i;
            }
        }
    }
}


void ParticleCollision::collisionWithinTriangles(const double& dt, const size_t i, std::vector<TrianglePtr> triangles, CollisionData& CD)
{
    double max_dti = 0.0;
    TrianglePtr collision_triangle = nullptr;
    Vector max_xi;
    bool collision_flag = true;
    int collision_num = 0;

    // loop over triangles
    for (auto& it: triangles)
    {
        // collision detection
        CollisionData collisionData;
        collisionDetection(dt, i, it, collisionData);
        if (collisionData.collision_status)
        {
            collision_num++;
            if (std::fabs(collisionData.dt_i) > std::fabs(max_dti))
            {
                max_dti = collisionData.dt_i;
                collision_triangle = it;
                max_xi = collisionData.x_i;
            }
        }
    }

    if (collision_num == 0) { collision_flag = false; } // no collision

    CD.collision_status = collision_flag;
    CD.triangle = collision_triangle;
    CD.x_i = max_xi;
    CD.dt_i = max_dti;
}


void ParticleCollision::collisionDetection(const double& dt, const size_t p, TrianglePtr triangle, CollisionData& CD)
{
    CD.triangle = triangle;
    CD.id = p;
    CD.collision_status = false;

    // detect collision with the plane
    double f0 = (DS->pos(p) - triangle->getP0()) * triangle->getNorm();
    if (f0 == 0.0)  { return;}
    double f1 = (DS->pos(p) - DS->vel(p) * dt - triangle->getP0()) * triangle->getNorm();
    if (f0 * f1 > 0.0)  { return;}
    // calculate dt_i and x_i
    double dti = ((DS->pos(p) - triangle->getP0()) * triangle->getNorm()) / (DS->vel(p) * triangle->getNorm());
    Vector xi = DS->pos(p) - DS->vel(p) * dti;

    // detect collision in the triangle
    Vector e1 = triangle->getE1();
    Vector e2 = triangle->getE2();

    double a = (e2^e1) * (e2^(xi - triangle->getP0())) / pow((e2^e1).magnitude(), 2.0);
    if (a < 0.0 || a > 1.0) { return;}
    double b = (e1^e2) * (e1^(xi - triangle->getP0())) / pow((e1^e2).magnitude(), 2.0);
    if (b < 0.0 || b > 1.0) { return;}
    double c = a + b;
    if (c < 0.0 || c > 1.0) { return;}

    if ((dt * dti) < 0.0)    { return;}
    if (fabs(dti) > fabs(dt))  { return;}
    if (((dt - dti) / dt) < pow(10.0, -6.0)) { return;}

    // parse interaction position and time
    CD.dt_i = dti;
    CD.x_i = xi;
    CD.collision_status = true;
}


void ParticleCollision::collisionHandling(const size_t p, const CollisionData& CD)
{
    // calculate vel_r and new pos
    Vector vel = DS->vel(p);
    Vector norm = CD.triangle->getNorm();
    Vector tmp = norm * (norm * vel);
    Vector vel_perp = vel - tmp;
    float Cs = float_parms.at("Cs");
    float Cr = float_parms.at("Cr");
    Vector vel_r = Cs * vel_perp - Cr * tmp;

    // update dynamical state
    Vector pos_new = CD.x_i + vel_r * CD.dt_i;
    DS->set_pos(p, pos_new);
    DS->set_vel(p, vel_r);
}

pba::CollisionPtr pba::CreateParticleCollision(DynamicalState ds)
{
    return CollisionPtr(new ParticleCollision(ds));
}

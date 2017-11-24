//
// Created by jingcoz on 11/23/17.
//

# include "SphereCollision.h"

using namespace std;
using namespace pba;

void SphereCollision::collision(const double &dt)
{
    // loop over spheres
    size_t i;
    for (i = 0; i < sphereDS->nb(); ++i)
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

void SphereCollision::collisionWithKdTree(const double &dt)
{
    // loop over spheres
    for (size_t i = 0; i < sphereDS->nb(); ++i)
    {
        bool collision_flag = true;
        double tmp_dt = dt;
        while (collision_flag)
        {
            // search triangles
            Vector vec1 = sphereDS->pos(i);
            Vector vec0 = vec1 - sphereDS->vel(i) * tmp_dt;
            CollisionData CD = geom->getKdTree()->searchCollision(shared_from_this(), tmp_dt, sphereDS, i, vec0, vec1);
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


void SphereCollision::collisionWithinTriangles(const double &dt, const size_t i, std::vector<TrianglePtr> triangles,
                                               CollisionData &CD)
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

void SphereCollision::collisionDetection(const double &dt, const size_t p, TrianglePtr triangle, CollisionData &CD)
{
    CD.triangle = triangle;
    CD.id = p;
    CD.collision_status = false;

    Vector center = sphereDS->pos(p);
    float radius = sphereDS->radius(p);
    Vector P0 = triangle->getP0();
    Vector n = triangle->getNorm();

    // sphere-triangle intersection test
    double dt_tmp0 = (center - P0) * n;
    if (fabs(dt_tmp0) >= radius)  { return;}

    Vector vel = sphereDS->vel(p);
    double dt_tmp1 = vel * n;
    double dt0 = (dt_tmp0 + radius) / dt_tmp1;
    double dt1 = (dt_tmp0 - radius) / dt_tmp1;

    double dti = 0.0;
    if (dt * dt0 <= 0.0 && dt * dt1 <= 0.0) { return;}
    if (dt * dt0 <= 0.0 && dt * dt1 > 0.0)  { dti = dt1;}
    if (dt * dt0 > 0.0 && dt * dt1 <= 0.0)  { dti = dt0;}
    if (dt * dt0 > 0.0 && dt * dt1 > 0.0)   { dti = (dt0 > dt1) ? dt0 : dt1; }

    Vector xi0 = center - vel * dti + n * radius;
    Vector xi1 = center - vel * dti - n * radius;
    if (isInTriangle(triangle, xi0))
    {
        CD.collision_status = true;
        CD.x_i = xi0;
        CD.dt_i = dti;
        return;
    }

    if (isInTriangle(triangle, xi1))
    {
        CD.collision_status = true;
        CD.x_i = xi1;
        CD.dt_i = dti;
        return;
    }
}

void SphereCollision::collisionHandling(const size_t p, const CollisionData &CD)
{
    Vector vel = sphereDS->vel(p);
    Vector n = CD.triangle->getNorm();
    Vector vel_r = Cs * vel - (Cs + Cr) * n * (n * vel);
    Vector pos_r = sphereDS->pos(p) + vel_r * CD.dt_i;

    sphereDS->set_vel(p, vel_r);
    sphereDS->set_pos(p, pos_r);
}


bool SphereCollision::isInTriangle(TrianglePtr triangle, const Vector &xi)
{
    // collision detection with triangle
    Vector e1 = triangle->getE1();
    Vector e2 = triangle->getE2();

    double a = (e2^e1) * (e2^(xi - triangle->getP0())) / pow((e2^e1).magnitude(), 2.0);
    if (a < 0.0 || a > 1.0) { return false;}
    double b = (e1^e2) * (e1^(xi - triangle->getP0())) / pow((e1^e2).magnitude(), 2.0);
    if (b < 0.0 || b > 1.0) { return false;}
    double c = a + b;
    if (c < 0.0 || c > 1.0) { return false;}

    return true;
}


pba::CollisionPtr pba::CreateSphereCollision(SphereState ds)
{
    return CollisionPtr(new SphereCollision(ds));
}

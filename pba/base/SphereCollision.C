//
// Created by jingcoz on 11/23/17.
//

# include "SphereCollision.h"

using namespace std;
using namespace pba;

void SphereCollision::collision(const double &dt)
{
    // loop over spheres
    for (size_t i = 0; i < sphereDS->nb(); ++i)
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
    Vector P = triangle->getP0();
    Vector n = triangle->getNorm();

    // sphere-triangle intersection test
    double dt_tmp0 = (center - P) * n;
    if (fabs(dt_tmp0) >= radius)  { return;}

    // sphere-trianglePlane intersection test
    triPlaneIntersectionTest(dt, p, triangle, CD);
    if (CD.collision_status)    { return;}

    // sphere-triangleEdge intersection test
    Vector P0 = triangle->getP0();
    Vector P1 = triangle->getP1();
    Vector P2 = triangle->getP2();
    CD.dt_i = 0.0;
    edgeIntersectionTest(dt, p, P0, P1, CD);
    edgeIntersectionTest(dt, p, P0, P2, CD);
    edgeIntersectionTest(dt, p, P1, P2, CD);
}

void SphereCollision::collisionHandling(const size_t p, const CollisionData &CD)
{
    // move to intersection position
    Vector center_intersect = sphereDS->pos(p) - CD.dt_i * sphereDS->vel(p);
    sphereDS->set_pos(p, center_intersect);

    // calculate refected velocity and position
    Vector vel = sphereDS->vel(p);
    Vector n = CD.triangle->getNorm();
    Vector vel_r = Cs * vel - (Cs + Cr) * n * (n * vel);
    Vector pos_r = sphereDS->pos(p) + vel_r * CD.dt_i;

    sphereDS->set_vel(p, vel_r);
    sphereDS->set_pos(p, pos_r);
}


void SphereCollision::triPlaneIntersectionTest(const double &dt, const size_t p, TrianglePtr triangle,
                                               CollisionData &CD)
{
    Vector center = sphereDS->pos(p);
    float radius = sphereDS->radius(p);
    Vector P0 = triangle->getP0();
    Vector n = triangle->getNorm();

    Vector vel = sphereDS->vel(p);
    double dt_tmp0 = (center - P0) * n;
    double dt_tmp1 = vel * n;
    double dt0 = (dt_tmp0 + radius) / dt_tmp1;
    double dt1 = (dt_tmp0 - radius) / dt_tmp1;

    // time test
    double dti = 0.0;
    bool dt0_test = timeTest(dt0, dt);
    bool dt1_test = timeTest(dt1, dt);
    if (!dt0_test && !dt1_test) { return;}
    dti = (fabs(dt0) * dt0_test > fabs(dt1) * dt1_test) ? dt0 : dt1;

    Vector xi0 = center - vel * dti + n * radius;
    Vector xi1 = center - vel * dti - n * radius;
    if (inTriangleTest(triangle, xi0))
    {
        CD.collision_status = true;
        CD.dt_i = dti;
        return;
    }

    if (inTriangleTest(triangle, xi1))
    {
        CD.collision_status = true;
        CD.dt_i = dti;
        return;
    }
}

void SphereCollision::edgeIntersectionTest(const double &dt, const size_t p, const Vector& P0, const Vector& P1, CollisionData &CD)
{
    Vector center = sphereDS->pos(p);
    float radius = sphereDS->radius(p);
    Vector vel = sphereDS->vel(p);
    Vector e = P1 - P0;

    double q = e * (center - P0) / (e * e);
    if (q > 1 || q < 0) { return;}

    double q_delta = pow(e * (center - P0), 2.0) + (e * e) * (radius * radius - (center - P0) * (center - P0));
    if (q_delta < 0.0)  { return;}  // no intersection with the edge line: no real roots

    Vector vel_perp = (vel - e * (e * vel)) / (e * e);
    Vector center_perp = (center - P0) - (e * ((center - P0) * e)) / (e * e);
    double vel_perp_sq = vel_perp * vel_perp;
    double center_perp_sq = center_perp * center_perp;
    double vel_perp_center_perp = vel_perp * center_perp;
    double dt_delta = pow(vel_perp_center_perp, 2.0) + vel_perp_sq * (center_perp_sq - radius * radius);
    if (dt_delta < 0.0) { return;}  // quadratic for dt has no real roots

    double dtmp0 = (-vel_perp_center_perp) / vel_perp_sq;
    double dtmp1 = pow(dt_delta, 0.5) / vel_perp_sq;
    double dt0 = dtmp0 + dtmp1;
    double dt1 = dtmp0 - dtmp1;
    // time test
    double dti = 0.0;
    bool dt0_test = timeTest(dt0, dt);
    bool dt1_test = timeTest(dt1, dt);
    if (!dt0_test && !dt1_test) { return;}
    dti = (fabs(dt0) * dt0_test > fabs(dt1) * dt1_test) ? dt0 : dt1;

    Vector xi = P0 + q * e;
    if (!CD.collision_status)
    {
        CD.collision_status = true;
        CD.dt_i = dti;
    }
    else    { if (fabs(dti) > fabs(CD.dt_i)) { CD.dt_i = dti; }}    // find the ealiest intersection
}


bool SphereCollision::inTriangleTest(TrianglePtr triangle, const Vector &xi)
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

bool SphereCollision::timeTest(const double &dti, const double &dt)
{
    if ((dt * dti) < 0.0)    { return false;}
    if (fabs(dti) > fabs(dt))  { return false;}
    if (((dt - dti) / dt) < pow(10.0, -6.0)) { return false;}
    return true;
}


//! ----------------------------------------------- create shared ptr -------------------------------------------------

pba::CollisionPtr pba::CreateSphereCollision(SphereState ds)
{
    return CollisionPtr(new SphereCollision(ds));
}

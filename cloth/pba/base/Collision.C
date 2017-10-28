//
// Created by jingcoz on 9/18/17.
//

# include <iostream>
# include "Collision.h"

using namespace std;
using namespace pba;

void TriangleCollision::collisionWithinTriangles(const double& dt, DynamicalState DS, size_t i, std::vector<TrianglePtr> triangles, CollisionData& CD)
{
    double max_dti = 0.0;
    TrianglePtr collision_triangle = NULL;
    Vector max_xi;
    bool collision_flag = true;
    int collision_num = 0;

    // loop over triangles
    for (auto it = triangles.cbegin(); it != triangles.cend(); ++it)
    {
        // collision detection
        TrianglePtr triangle = *it;
        double dti;
        Vector xi;
        if (collisionDetection(dt, DS, i, triangle, dti, xi))
        {
            collision_num++;
            if (std::fabs(dti) > std::fabs(max_dti))
            {
                max_dti = dti;
                collision_triangle = triangle;
                max_xi = xi;
            }
        }
    }

    if (collision_num == 0) { collision_flag = false; } // no collision

    CD.collision_status = collision_flag;
    CD.triangle = collision_triangle;
    CD.x_i = max_xi;
    CD.dt_i = max_dti;
}

void TriangleCollision::triangleCollision(const double& dt, DynamicalState DS, GeometryPtr geom, const double& Cr, const double& Cs)
{
    // loop over particles
    size_t i;
    // #pragma omp parallel for num_threads(4) private(i) schedule(dynamic, 256)
    for (i = 0; i < DS->nb(); ++i)
    {
        // loop until no collisions
        bool collision_flag = true;
        double tmp_dt = dt;
        while (collision_flag)
        {
            CollisionData CD;
            collisionWithinTriangles(tmp_dt, DS, i, geom->get_triangles(), CD);
            collision_flag = CD.collision_status;
            if (collision_flag)
            {
                // handle collision for maximum dt_i
                TriangleCollision::collisionHandling(tmp_dt, DS, i, CD, Cr, Cs);
                // store collision triangle
                CD.triangle->setCollisionStatus(true);
                // update dt (use maximum dti as new dt for next collision detection)
                tmp_dt = CD.dt_i;
            }
        }
    }
}

void TriangleCollision::triangleCollisionWithKdTree(const double& dt, DynamicalState DS, GeometryPtr geom, const double& Cr, const double& Cs)
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
            CollisionData CD = geom->getKdTree()->searchCollision(tmp_dt, DS, i, vec0, vec1);
            collision_flag = CD.collision_status;

            if (collision_flag)
            {
                // handle collision for maximum dt_i
                TriangleCollision::collisionHandling(tmp_dt, DS, i, CD, Cr, Cs);
                // store collision triangle
                CD.triangle->setCollisionStatus(true);
                // update dt (use maximum dti as new dt for next collision detection)
                tmp_dt = CD.dt_i;
            }
        }
    }
}


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


void TriangleCollision::collisionHandling(const double& dt, DynamicalState DS, const size_t p, const CollisionData& CD, const double& Cr, const double& Cs)
{
    // calculate vel_r and new pos
    Vector vel = DS->vel(p);
    Vector norm = CD.triangle->getNorm();
    Vector tmp = norm * (norm * vel);
    Vector vel_perp = vel - tmp;
    Vector vel_r = Cs * vel_perp - Cr * tmp;

    // update dynamical state
    Vector pos_new = CD.x_i + vel_r * CD.dt_i;
    DS->set_pos(p, pos_new);
    DS->set_vel(p, vel_r);
}

//! --------------------------------------------------------------------------------------------------------------------


void RBDCollision::RBD_Collision(const double &dt,  const RigidBodyState& RBDS, pba::GeometryPtr geom)
{
    bool collision_flag = true;
    double tmp_dt = dt;
    while (collision_flag)
    {
        CollisionData CD;
        collision(tmp_dt, RBDS, geom->get_triangles(), CD);
        if (CD.collision_status)
        {
            // handle collision for maximum dt_i
            collisionHandling(RBDS, CD);
            // store collision triangle
            CD.triangle->setCollisionStatus(true);

            tmp_dt = CD.dt_i;
        }
        else {collision_flag = false;}
    }
}

void RBDCollision::collision(const double &dt, const RigidBodyState &RBDS, std::vector<TrianglePtr> triangles, CollisionData &CD)
{
    double max_dti = 0.0;
    TrianglePtr collision_triangle = nullptr;
    bool collision_flag = true;
    int collision_num = 0;
    size_t id = 0;

    // loop over particles
    for (size_t i = 0 ; i < RBDS->nb(); ++i)
    {
        // loop over triangles
        for (auto it = triangles.cbegin(); it != triangles.cend(); ++it)
        {
            // collision detection
            TrianglePtr triangle = *it;
            double dti;
            if (collisionDetection(dt, RBDS, i, triangle, dti))
            {
                collision_num++;
                if (std::fabs(dti) > std::fabs(max_dti))    // keep largest dti and which triangle
                {
                    max_dti = dti;
                    collision_triangle = triangle;
                    id = i;
                }
            }
        }
    }

    if (collision_num == 0) { collision_flag = false; } // no collision

    CD.collision_status = collision_flag;
    CD.triangle = collision_triangle;
    CD.dt_i = max_dti;
    CD.id = id;
}

bool RBDCollision::collisionDetection(const double &dt,  const RigidBodyState& RBDS, const size_t& p, TrianglePtr triangle, double &dt_i)
{
    // detect collision with the plane
    Vector vert_pos = RBDS->vert_pos(p);
    double f0 = (vert_pos - triangle->getP0()) * triangle->getNorm();
    if (f0 == 0.0)  { return false;}

    Vector vel_ang = RBDS->get_vel_angular();
    Vector vel_ang_unit = vel_ang.unitvector();
    double vel_ang_mag = vel_ang.magnitude();

    Matrix U = rotation(vel_ang_unit, vel_ang_mag * dt) * RBDS->get_angular_rotation();
    Vector tmp_x = U * RBDS->pos(p) + RBDS->get_pos_cm() - RBDS->get_vel_cm() * dt;
    double f1 = ( tmp_x - triangle->getP0()) * triangle->getNorm();
    if (f0 * f1 > 0.0)  { return false;}
    if (f1 == 0.0)
    {
        dt_i = dt;
        return true;
    }

    // get converged
    double t0 = 0.0;
    double t1 = dt;
    double dti;
    double ff;
    bool converged = false;
    while (!converged)
    {
        dti = (t0 + t1) / 2.0;
        U = rotation(vel_ang_unit, vel_ang_mag * dti) * RBDS->get_angular_rotation();
        tmp_x = U * RBDS->pos(p) + RBDS->get_pos_cm() - RBDS->get_vel_cm() * dti;
        ff = ( tmp_x - triangle->getP0()) * triangle->getNorm();
        if (ff == 0.0) { converged = true;  continue;}

        if (ff * f0 < 0.0)
        {
            f1 = ff;
            t1 = dti;
        } else
        {
            f0 = ff;
            t0 = dti;
        }
        if (fabs((t0 - t1) / dt) < 0.001)   {converged = true;}
    }
    if (ff * f0 > 0.0)  {dti = t1;}

    // collision detection with triangle
    Vector xi = U * RBDS->pos(p) + RBDS->get_pos_cm() - RBDS->get_vel_cm() * dti;
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

    dt_i = dti;
    return true;
}

void RBDCollision::collisionHandling(const RigidBodyState &RBDS, const CollisionData &CD)
{
    // move to rigid body state to the collision moment
    double dti = CD.dt_i;
    Vector pos_cm = RBDS->get_pos_cm() - RBDS->get_vel_cm() * dti;
    Vector vel_ang = RBDS->get_vel_angular();
    Matrix R = rotation(vel_ang.unitvector(), vel_ang.magnitude() * dti) * RBDS->get_angular_rotation();

    RBDS->set_pos_cm(pos_cm);
    RBDS->set_angular_rotation(R);
    RBDS->set_moment_of_inertia();

    // modify v_cm and w to reflect off the surface
    size_t p = CD.id;
    Matrix I = RBDS->get_moment_of_inertia();
    Matrix I_inverse = I.inverse();
    Vector np = CD.triangle->getNorm();
    Vector q = I_inverse * (RBDS->vert_rel_pos(p) ^ np);

    Vector vel_cm = RBDS->get_vel_cm();
    double total_mass = RBDS->get_total_mass();
    double A0 = 2.0 * vel_cm * np + q * I * vel_ang + vel_ang * I * q;
    double A1 = (1.0 / total_mass) + q * I * q;
    double A = -A0 / A1;

    Vector vel_cm_ref = vel_cm + A * np / total_mass;
    Vector vel_ang_ref = vel_ang + A * q;

    RBDS->set_vel_cm(vel_cm_ref);
    RBDS->set_vel_angular(vel_ang_ref);

    // move the rigid body state by dti
    pos_cm = RBDS->get_pos_cm() + RBDS->get_vel_cm() * dti;
    vel_ang = RBDS->get_vel_angular();
    R = rotation(vel_ang.unitvector(), -vel_ang.magnitude() * dti) * RBDS->get_angular_rotation();
    RBDS->set_pos_cm(pos_cm);
    RBDS->set_angular_rotation(R);
    RBDS->set_moment_of_inertia();
}

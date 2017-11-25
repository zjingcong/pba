//
// Created by jingcoz on 11/23/17.
//

# include "SphereCollision.h"

using namespace std;
using namespace pba;


SphereCollision::SphereCollision(SphereState ds):
        sphereDS(ds)
{
    float_parms.insert({"sphere_Cr", 1.0});
    float_parms.insert({"sphere_Cs", 1.0});
}

void SphereCollision::init()
{
    size_t sphere_nb = sphereDS->nb();
    // init sphere collision detection flags
    detection_flags.resize(sphere_nb);
    for (size_t i = 0; i < sphere_nb; ++i)
    {
        detection_flags[i].resize(sphere_nb);
        for (size_t j = 0; j < sphere_nb; ++j) { detection_flags[i][j] = (i == j); }
    }
}

void SphereCollision::collision(const double &dt)
{
    triCollision(dt);
    sphereCollision(dt);
    triCollision(dt);
}

void SphereCollision::collisionWithKdTree(const double &dt)
{
    triCollisionWithKdTree(dt);
    sphereCollision(dt);
    triCollisionWithKdTree(dt);
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
        triCollisionDetection(dt, i, it, collisionData);
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

//! ---------------------------------------- collision between sphere and triangle -----------------------------------

void SphereCollision::triCollision(const double &dt)
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
                triCollisionHandling(i, CD);
                // store collision triangle
                CD.triangle->setCollisionStatus(true);
                // update dt (use maximum dti as new dt for next collision detection)
                tmp_dt = CD.dt_i;
            }
        }
    }
}

void SphereCollision::triCollisionWithKdTree(const double &dt)
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
                triCollisionHandling(i, CD);
                // store collision triangle
                CD.triangle->setCollisionStatus(true);
                // update dt (use maximum dti as new dt for next collision detection)
                tmp_dt = CD.dt_i;
            }
        }
    }
}

void SphereCollision::triCollisionDetection(const double &dt, const size_t p, TrianglePtr triangle, CollisionData &CD)
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
    if (CD.collision_status)    { return;}

    // sphere-triangleVertex intersection test
    CD.dt_i = 0.0;
    vertIntersectionTest(dt, p, P0, CD);
    vertIntersectionTest(dt, p, P1, CD);
    vertIntersectionTest(dt, p, P2, CD);
}

void SphereCollision::triCollisionHandling(const size_t p, const CollisionData &CD)
{
    // move to intersection position
    Vector center_intersect = sphereDS->pos(p) - CD.dt_i * sphereDS->vel(p);
    sphereDS->set_pos(p, center_intersect);

    // calculate refected velocity and position
    Vector vel = sphereDS->vel(p);
    Vector n = CD.triangle->getNorm();
    float Cs = float_parms.at("Cs");
    float Cr = float_parms.at("Cr");
    Vector vel_r = Cs * vel - (Cs + Cr) * n * (n * vel);
    Vector pos_r = sphereDS->pos(p) + vel_r * CD.dt_i;

    sphereDS->set_vel(p, vel_r);
    sphereDS->set_pos(p, pos_r);
}

//! ---------------------------------------- collision between sphere and sphere -----------------------------------

void SphereCollision::sphereCollision(const double &dt)
{
    reset_detect_flags();
    size_t spbere_nb = sphereDS->nb();
    // loop over spheres
    for (size_t i = 0; i < spbere_nb; ++i)
    {
        double tmp_dt = dt;
        SphereCollisionData sphereCollisionData;
        sphereCollisionData.collision_status = true;

        while (sphereCollisionData.collision_status)
        {
            double max_dti = 0.0;
            size_t coll_sphere = 0;
            int collision_num = 0;
            bool collision_flag = true;
            for (size_t j = 0; j < spbere_nb; ++j)  // TODO: advanced-use grid data structure and loop over neighbor spheres
            {
                SphereCollisionData sphereCD;
                if (!detection_flags[i][j])
                {
                    sphereCollisionDetection(tmp_dt, i, j, sphereCD);
//                detection_flags[i][j] = true;
//                detection_flags[j][i] = true;
                    if (sphereCD.collision_status)
                    {
                        collision_num++;
                        if (fabs(sphereCD.dt_i) > fabs(max_dti))
                        {
                            max_dti = sphereCD.dt_i;
                            coll_sphere = sphereCD.id2;
                        }
                    }
                }
                if (collision_num == 0) {collision_flag = false;}
            }

            sphereCollisionData.id1 = i;
            sphereCollisionData.id2 = coll_sphere;
            sphereCollisionData.collision_status = collision_flag;
            sphereCollisionData.dt_i = max_dti;
            if (sphereCollisionData.collision_status)
            {
                sphereCollisionHandling(i, sphereCollisionData);
                tmp_dt = sphereCollisionData.dt_i;
            }
        }
    }
}

void SphereCollision::sphereCollisionDetection(const double &dt, const size_t i, const size_t j, SphereCollisionData& sphereCD)
{
    sphereCD.id1 = i;
    sphereCD.id2 = j;
    sphereCD.collision_status = false;

    Vector s1 = sphereDS->pos(i);
    Vector s2 = sphereDS->pos(j);
    float r1 = sphereDS->radius(i);
    float r2 = sphereDS->radius(j);

    if ((s1 - s2).magnitude() > (r1 + r2))  { return;}  // no collision

    Vector v1 = sphereDS->vel(i);
    Vector v2 = sphereDS->vel(j);

    Vector tmp_s = s1 - s2;
    Vector tmp_v = v2 - v1;
    float tmp_r = r1 + r2;
    double s_sq = tmp_s * tmp_s;
    double v_sq = tmp_v * tmp_v;
    float r_sq = tmp_r * tmp_r;
    double dt_delta = s_sq * v_sq + v_sq * (r_sq - s_sq);
    double dt0 = (-tmp_s * tmp_v + pow(dt_delta, 0.5)) / v_sq;
    double dt1 = (-tmp_s * tmp_v - pow(dt_delta, 0.5)) / v_sq;
    // time test
    double dti = 0.0;
    bool dt0_test = timeTest(dt0, dt);
    bool dt1_test = timeTest(dt1, dt);
    if (!dt0_test && !dt1_test) { return;}
    dti = (fabs(dt0) * dt0_test > fabs(dt1) * dt1_test) ? dt0 : dt1;

    sphereCD.collision_status = true;
    sphereCD.dt_i = dti;
    sphereDS->set_isCollision(i, 1);
    sphereDS->set_isCollision(j, 1);
}

void SphereCollision::sphereCollisionHandling(const size_t i, SphereCollisionData &sphereCD)
{
    size_t j = sphereCD.id2;
    Vector s1 = sphereDS->pos(i);
    Vector s2 = sphereDS->pos(j);
    Vector v1 = sphereDS->vel(i);
    Vector v2 = sphereDS->vel(j);
    float m1 = sphereDS->mass(i);
    float m2 = sphereDS->mass(j);
    float total_mass = m1 + m2;
    double dti = sphereCD.dt_i;

    // move to point of contact
    Vector new_s1 = s1 - v1 * dti;
    Vector new_s2 = s2 - v2 * dti;
    sphereDS->set_pos(i, new_s1);
    sphereDS->set_pos(j, new_s2);
    // calculate reflected velocity and position
    s1 = sphereDS->pos(i);
    s2 = sphereDS->pos(j);
    Vector v_cm = (m1 * v1 + m2 * v2) / total_mass;
    Vector v_rel = v2 - v1; // related velocity
    Vector norm = (s1 - s2).unitvector();
    float sphere_Cr = float_parms.at("sphere_Cr");
    float sphere_Cs = float_parms.at("sphere_Cs");
    Vector v_ref_rel = sphere_Cs * (v_rel) - (sphere_Cr + sphere_Cs) * norm * (norm * v_rel);
    Vector v1_ref = v_cm - (m2 * v_ref_rel) / total_mass;
    Vector v2_ref = v_cm - (m1 * v_ref_rel) / total_mass;
    new_s1 = s1 + v1_ref * dti;
    new_s2 = s2 + v2_ref * dti;
    // set reflected velocity and position
    sphereDS->set_pos(i, new_s1);
    sphereDS->set_pos(j, new_s2);
    sphereDS->set_vel(i, v1_ref);
    sphereDS->set_vel(j, v2_ref);
}

//! --------------------------------------------- collision tests --------------------------------------------------

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
    // find the ealiest intersection
    else    { if (fabs(dti) > fabs(CD.dt_i)) { CD.dt_i = dti; }}
}

void SphereCollision::vertIntersectionTest(const double &dt, const size_t p, const Vector &P, CollisionData &CD)
{
    Vector center = sphereDS->pos(p);
    float radius = sphereDS->radius(p);
    Vector vel = sphereDS->vel(p);

    double tmp = vel * (center - P);
    double vel_sq = vel * vel;
    double dt_delta = pow(tmp, 2.0) - vel_sq * ((center - P) * (center - P) - radius * radius);
    if (dt_delta < 0.0) { return;}  // quadratic for dt has no real roots

    double dt0 = (tmp + pow(dt_delta, 0.5)) / vel_sq;
    double dt1 = (tmp - pow(dt_delta, 0.5)) / vel_sq;

    // time test
    double dti = 0.0;
    bool dt0_test = timeTest(dt0, dt);
    bool dt1_test = timeTest(dt1, dt);
    if (!dt0_test && !dt1_test) { return;}
    dti = (fabs(dt0) * dt0_test > fabs(dt1) * dt1_test) ? dt0 : dt1;

    if (!CD.collision_status)
    {
        CD.collision_status = true;
        CD.dt_i = dti;
    }
    // find the ealiest intersection
    else    { if (fabs(dti) > fabs(CD.dt_i)) { CD.dt_i = dti; }}
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


void SphereCollision::reset_detect_flags()
{
    for (size_t i = 0; i < detection_flags.size(); ++i)
    {
        for (size_t j = 0; j < detection_flags.size(); ++j)
        {
            detection_flags[i][j] = (i == j);
        }
    }
}


//! ----------------------------------------------- create shared ptr -------------------------------------------------

pba::CollisionPtr pba::CreateSphereCollision(SphereState ds)
{
    return CollisionPtr(new SphereCollision(ds));
}
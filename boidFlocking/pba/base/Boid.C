//
// Created by jingcoz on 10/2/17.
//

# include "Boid.h"

using namespace std;
using namespace pba;

Boid::Boid(DynamicalState& DS):
        boidDS(DS),
        Ka(0.0),
        Kv(0.0),
        Kc(0.0),
        r1(100.0),
        r2(100.0),
        theta1(360.0),
        theta2(360.0),
        accel_max(10000)
{}

Vector Boid::guide(const size_t i)
{
    Vector guiding_force = Vector(0.0, 0.0, 0.0);
    for (size_t p = 0; p < guidingForces->size(); p++)
    {
        Vector locator = guidingLocators->at(p);
        Vector d = locator - boidDS->pos(i);
        Vector vel = boidDS->vel(i);
        double Kv = vision(d, vel);
        guiding_force += (guidingForces->at(p)->getForce(boidDS, i)) * Kv;
    }
    Vector accel_g = guiding_force / boidDS->mass(i);

    return accel_g;
}

double Boid::rangeWeight(const double &d)
{
    if (d <= r1)    {return 1.0;}
    if (d >= r2)    { return 0.0;}
    double Kr = (r2 - d) / (r2 - r1);
    return Kr;
}

double Boid::vision(const Vector &d, const Vector &vel)
{
    double cos_theta = (vel * d) / (vel.magnitude() * d.magnitude());
    double cos_half_theta1 = std::cos(M_PI * (theta1 * 0.5) / 180.0);
    double cos_half_theta2 = std::cos(M_PI * (theta2 * 0.5) / 180.0);
    if (cos_theta >= cos_half_theta1)   { return 1.0;}
    if (cos_theta <= cos_half_theta2)   { return 0.0;}
    double K_theta = (cos_half_theta2 - cos_theta) / (cos_half_theta2 - cos_half_theta1);
    return K_theta;
}

void Boid::accelThreshold(Vector &accel_a, Vector &accel_v, Vector &accel_c, Vector& accel_g)
{
    double aa_mag = accel_a.magnitude();
    if (aa_mag > accel_max)
    {
        accel_a = accel_max * (accel_a).unitvector();
        accel_v = Vector(0.0, 0.0, 0.0);
        accel_c = Vector(0.0, 0.0, 0.0);
        accel_g = Vector(0.0, 0.0, 0.0);
        return;
    }
    double residual = accel_max - aa_mag;

    // add guiding accel
    double ag_mag = accel_g.magnitude();
    if (ag_mag > residual)
    {
        accel_g = residual * (accel_g).unitvector();
        accel_v = Vector(0.0, 0.0, 0.0);
        accel_c = Vector(0.0, 0.0, 0.0);
        return;
    }
    residual = residual - ag_mag;

    double av_mag = accel_v.magnitude();
    if (av_mag > residual)
    {
        accel_v = residual * (accel_v).unitvector();
        accel_c = Vector(0.0, 0.0, 0.0);
        return;
    }
    residual = residual - av_mag;
    if (accel_c.magnitude() > residual)
    {
        accel_c = residual * (accel_c).unitvector();
        return;
    }
}

Vector Boid::get_total_accel(const size_t i)
{
    Vector accel_a = Vector(0.0, 0.0, 0.0);
    Vector accel_v = Vector(0.0, 0.0, 0.0);
    Vector accel_c = Vector(0.0, 0.0, 0.0);
    Vector accel_g = Vector(0.0, 0.0, 0.0);
    Vector X_i = boidDS->pos(i);
    Vector vel_i = boidDS->vel(i);
    for (size_t j = 0; j < boidDS->nb(); j++)
    {
        if (j != i)
        {
            Vector d = boidDS->pos(j) - X_i;
            double d_mag = d.magnitude();

            double Kr = rangeWeight(d_mag);
            double K_theta = vision(d, vel_i);

            // collision avoidance
            accel_a += (-Ka * d / pow(d_mag, 2.0)) * Kr * K_theta;
            // velocity matching
            accel_v += (Kv * (boidDS->vel(j) - vel_i)) * Kr * K_theta;
            // centering
            accel_c += (Kc * d) * Kr * K_theta;

            if (d_mag == 0.0)   // handle special case, usually won't happen
            {
                accel_a = accel_max * d.unitvector();
                accel_v = Vector(0.0, 0.0, 0.0);
                accel_c = Vector(0.0, 0.0, 0.0);
            }
        }
    }
    // guiding
    accel_g = guide(i);

    // limit the accel and prioritize basic behaviors
    accelThreshold(accel_a, accel_v, accel_c, accel_g);

    // get total acceleration
    Vector total_accel = accel_a + accel_v + accel_c + accel_g;

    return total_accel;
}

//
// Created by jingcoz on 10/2/17.
//

# include "Boid.h"

using namespace std;
using namespace pba;

Boid::Boid(DynamicalState DS):
        boidDS(DS),
        r1(0.0),
        r2(0.0),
        theta1(0.0),
        theta2(0.0),
        accel_max(10000)
{}


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
    double cos_half_theta1 = std::cos(M_PI * theta1 / 180.0);
    double cos_half_theta2 = std::cos(M_PI * theta2 / 180.0);
    if (cos_theta >= cos_half_theta1)   { return 1.0;}
    if (cos_theta <= cos_half_theta2)   { return 0.0;}
    double K_theta = (cos_half_theta2 - cos_theta) / (cos_half_theta2 - cos_half_theta1);
    return K_theta;
}

void Boid::accelThreshold(Vector &accel_a, Vector &accel_v, Vector &accel_c)
{
    double aa_mag = accel_a.magnitude();
    if (aa_mag > accel_max)
    {
        accel_a = accel_max * (accel_a).unitvector();
        accel_v = Vector(0.0, 0.0, 0.0);
        accel_c = Vector(0.0, 0.0, 0.0);
        return;
    }
    double residual = accel_max - aa_mag;
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
    }
}

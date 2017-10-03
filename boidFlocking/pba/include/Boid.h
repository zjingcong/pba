//
// Created by jingcoz on 10/2/17.
//

# ifndef PBA_BOID_H
# define PBA_BOID_H

# include "assert.h"
# include "cmath"

# include "DynamicalState.h"
# include "Vector.h"

namespace pba
{

    class Boid
    {
    public:
        Boid(DynamicalState& DS);
        ~Boid() {}

        void set_Ka(const double& ka)   {Ka = ka;}
        void set_Kv(const double& kv)   {Kv = kv;}
        void set_Kc(const double& kc)   {Kc = kc;}
        void set_r1(const double& r)   {r1 = r; r2 = r1 + r_ramp;}
        void set_r_ramp(const double& r)    {r_ramp = r; assert(r >= 0.0); r2 = r1 + r_ramp;}
        void set_theta1(const double& theta)    {theta1 = theta; assert(checkTheta(theta1, theta2));}
        void set_theta2(const double& theta)    {theta2 = theta; assert(checkTheta(theta1, theta2));}
        void set_accel_max(const double& a) {accel_max = a; assert(accel_max > 0);}

        Vector get_total_accel(const size_t i);

    private:
        DynamicalState& boidDS;  // boid particles dynamical state
        //! basic attributes
        double Ka;  // avoidance strength
        double Kv;  // velocity matching strength
        double Kc;  // centering strength
        //! modifiers attributes
        // range
        double r1;
        double r2;
        // vision
        double theta1;  // measure in degrees
        double theta2;
        // threshold acceleration
        double accel_max;
        // ramp
        double r_ramp;

        double rangeWeight(const double& d);
        double vision(const Vector& d, const Vector& vel);
        void accelThreshold(Vector& accel_a, Vector& accel_v, Vector& accel_c);

        bool checkTheta(double& t1, double& t2)
        {
            double th1 = int(floor(t1)) % 720 + (t1 - floor(t1));
            double th2 = int(floor(t2)) % 720 + (t2 - floor(t2));
            return ((th2 >= th1) && (th1 >= 0) && (th1 <= 360) && (th2 >= 0) && (th2 <= 360));
        }
    };

    typedef Boid* BoidPtr;

}

# endif //PBA_BOID_H

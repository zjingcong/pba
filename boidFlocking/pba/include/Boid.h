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
        Boid(DynamicalState DS);
        ~Boid() {}

        void set_r1(const double& r)   {r1 = r; assert(r1 <= r2);}
        void set_r2(const double& r)    {r2 = r; assert(r1 <= r2);}
        void set_theta1(const double& theta)    {theta1 = theta; assert(std::fabs(theta1) <= std::fabs(theta2));}
        void set_theta2(const double& theta)    {theta2 = theta; assert(std::fabs(theta1) <= std::fabs(theta2));}
        void set_accel_max(const double& a) {accel_max = a; assert(accel_max > 0);}

    private:
        DynamicalState boidDS;  // boid particles dynamical state
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

        double rangeWeight(const double& d);
        double vision(const Vector& d, const Vector& vel);
        void accelThreshold(Vector& accel_a, Vector& accel_v, Vector& accel_c);
    };

}

# endif //PBA_BOID_H

//
// Created by jingcoz on 10/2/17.
//

# ifndef PBA_BOID_H
# define PBA_BOID_H

# include "assert.h"
# include "cmath"

# include "DynamicalState.h"
# include "Vector.h"
# include "Force.h"

namespace pba
{

    class Boid
    {
    public:
        Boid(DynamicalState& DS);
        ~Boid() {}

        //! set boid system attributes
        void set_Ka(const double& ka)   {Ka = ka;}
        void set_Kv(const double& kv)   {Kv = kv;}
        void set_Kc(const double& kc)   {Kc = kc;}
        void set_r1(const double& r)   {r1 = r; r2 = r1 + r_ramp;}
        void set_r_ramp(const double& r)    {r_ramp = r; assert(r >= 0.0); r2 = r1 + r_ramp;}
        void set_theta1(const double& theta)    {theta1 = theta; theta2 = theta1 + theta_ramp;}
        void set_theta_ramp(const double& theta)    {theta_ramp = theta; theta2 = theta1 + theta_ramp;}
        void set_accel_max(const double& a) {accel_max = a; assert(accel_max > 0);}
        //! set boid system guide
        void set_guiding_forces(ForcePtrContainer& force, std::vector<Vector>& l)
            {guidingForces = &force; guidingLocators = &l; assert(guidingForces->size() == guidingLocators->size());}

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
        double theta_ramp;
        //! steering and guiding
        ForcePtrContainer* guidingForces;
        std::vector<Vector>* guidingLocators;

        Vector guide(const size_t i);
        double rangeWeight(const double& d);
        double vision(const Vector& d, const Vector& vel);
        void accelThreshold(Vector& accel_a, Vector& accel_v, Vector& accel_c, Vector& accel_g);
    };

    /// set up boid shared ptr
    typedef std::shared_ptr<Boid> BoidPtr;
    BoidPtr CreateBoid(DynamicalState& DS);

}

# endif //PBA_BOID_H

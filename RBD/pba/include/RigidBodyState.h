//
// Created by jingcoz on 10/20/17.
//

#ifndef PBA_RIGIDBODYSTATE_H
#define PBA_RIGIDBODYSTATE_H

# include "DynamicalState.h"
# include "Matrix.h"
# include "LinearAlgebra.h"
# include "Force.h"
# include <tuple>

namespace pba
{

    class RigidBodyStateData: public DynamicalStateData, public std::enable_shared_from_this<RigidBodyStateData>
    {
    public:
        RigidBodyStateData(const std::string& nam = "RigidBodyDataNoName");
        ~RigidBodyStateData()   {}

        void Init(const std::vector<Vector> &x, const std::vector<double> &m, const Vector &v_cm, const Vector &v_ang);
        void Reset(const Vector &v_cm, const Vector &v_ang);

        const Vector vert_rel_pos(const size_t p) const;
        const Vector vert_pos(const size_t p) const;
        void set_moment_of_inertia();
        std::tuple<pba::Vector, pba::Vector> totalForce_and_tau(pba::ForcePtrContainer& forces);

        const double& get_total_mass() const    { return total_mass;}
        const Vector& get_pos_cm() const    { return pos_cm;}
        const Vector& get_vel_cm() const    { return vel_cm;}
        const Vector& get_vel_angular() const { return vel_angular;}
        const Matrix& get_angular_rotation() const { return angular_rotation;}
        const Matrix& get_moment_of_inertia() const { return moment_of_inertia;}

        void set_pos_cm(const Vector& p)    {pos_cm = p;}
        void set_vel_cm(const Vector& vel)  {vel_cm = vel;}
        void set_vel_angular(const Vector& vel) {vel_angular = vel;}
        void set_angular_rotation(const Matrix& m)  {angular_rotation = m;}


    private:
        double total_mass;
        Matrix moment_of_inertia;
        Vector pos_cm;
        Matrix angular_rotation;
        Vector vel_cm;
        Vector vel_angular;
        Vector init_pos_cm;

        void set_total_mass();
        void set_pos_cm(const std::vector<Vector>& x);
        void set_pi(const std::vector<Vector>& x);
    };

    typedef std::shared_ptr<RigidBodyStateData> RigidBodyState;
    RigidBodyState CreateRigidBodyState(const std::string& nam = "RigidBodyDataNoName");

}

#endif //PBA_RIGIDBODYSTATE_H

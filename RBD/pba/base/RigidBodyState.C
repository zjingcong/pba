//
// Created by jingcoz on 10/20/17.
//

# include "RigidBodyState.h"
# include <cassert>
# include <iostream>

using namespace pba;
using namespace std;


RigidBodyStateData::RigidBodyStateData(const std::string &nam): DynamicalStateData(nam)
{
    nb_items = 0;
    name = nam;
    total_mass = 0.0;
    moment_of_inertia = unitMatrix();
    pos_cm = Vector(0.0, 0.0, 0.0);
    angular_rotation = unitMatrix();
    vel_cm = Vector(0.0, 0.0, 0.0);
    vel_angular = Vector(0.0, 0.0, 0.0);
}

void RigidBodyStateData::set_init(const std::vector<Vector>& x, const std::vector<double>& m, const Vector& v_cm, const Vector& v_ang)
{
    // init nb_item
    assert(x.size() == m.size());
    cout << "Init rigid body state data..." << endl;
    add(x.size());
    // init mass
    for (size_t i = 0; i < nb_items; ++i)
    {
        double mass = m[i];
        set_mass(i, float(mass));
    }
    // RB init
    set_total_mass();
    set_pos_cm();
    vel_cm = v_cm;
    vel_angular = v_ang;
    set_pi(x);
    cout << "Complete rigid body state data initialization." << endl;
}

const Vector RigidBodyStateData::vert_rel_pos(const size_t p) const
{
    Vector pi = pos(p);
    return angular_rotation * pi;
}

const Vector RigidBodyStateData::vert_pos(const size_t p) const
{
    Vector ri = vert_rel_pos(p);
    return pos_cm + ri;
}

void RigidBodyStateData::set_moment_of_inertia()
{
    for (int m = 0; m < 3; ++m)
    {
        for (int n = 0; n < 3; ++n)
        {
            double delta = (m == n) ? 1.0: 0.0;
            double value = 0.0;
            for (size_t i = 0; i < nb_items; ++i)
            {
                Vector ri = vert_rel_pos(i);
                double tmp = mass(i) * (pow(ri.magnitude(), 2.0) * delta - ri[m] * ri[n]);
                value += tmp;
            }
            moment_of_inertia.Set(m, n, value);
        }
    }
}


// ---------------------------------------------------------------------------------------------------

void RigidBodyStateData::set_total_mass()
{
    double m = 0.0;
    for (size_t i = 0; i < nb_items; ++i)
    {
        m += mass(i);
    }
    total_mass = m;
}

void RigidBodyStateData::set_pos_cm()
{
    Vector l = Vector(0.0, 0.0, 0.0);
    for (size_t i = 0; i < nb_items; ++i)
    {
        l += (mass(i) * pos(i));
    }
    pos_cm = l / total_mass;    // run after set_total_mass()
}

void RigidBodyStateData::set_pi(const std::vector<Vector>& x)
{
    for (size_t i = 0; i < nb_items; ++i)
    {
        Vector pi = x[i] - pos_cm;
        set_pos(i, pi);         // run after set_pos_cm()
    }
}


pba::RigidBodyState pba::CreateRigidBodyState(const std::string &nam)
{
    return pba::RigidBodyState(new RigidBodyStateData(nam));
}


std::tuple<pba::Vector, pba::Vector> pba::totalForce_and_tau(pba::ForcePtrContainer& forces, const pba::RigidBodyState &RBDS)
{
    Vector total_force = Vector(0.0, 0.0, 0.0);
    Vector tau = Vector(0.0, 0.0, 0.0);
    for (size_t i = 0; i < RBDS->nb(); ++i)
    {
        Vector force_value = Vector(0.0, 0.0, 0.0);
        for (auto& it: forces)
        {
            force_value += it->getForce(RBDS, i);
        }
        total_force += force_value;
        tau += RBDS->vert_rel_pos(i) ^ force_value;
    }

    return std::make_tuple(total_force, tau);
}

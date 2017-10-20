//
// Created by jingcoz on 10/20/17.
//

# include "RBD.h"

using namespace pba;
using namespace std;

void RBDSolverBase::updatePos(const double& dt, const RigidBodyState& RBDS)
{
    Vector pos_cm = RBDS->get_vel_cm() * dt + RBDS->get_pos_cm();
    Vector v_ang = RBDS->get_vel_angular();
    Matrix R = rotation(v_ang.unitvector(), -v_ang.magnitude() * dt) * RBDS->get_angular_rotation();

    RBDS->set_pos_cm(pos_cm);
    RBDS->set_angular_rotation(R);
    RBDS->set_moment_of_inertia();
}

void RBDSolverBase::updateVel(const double &dt, const RigidBodyState& RBDS, ForcePtrContainer& forces)
{
    Vector total_force;
    Vector tau;
    std::tie(total_force, tau) = totalForce_and_tau(forces, RBDS);
    Vector vel_cm = total_force * dt / (RBDS->get_total_mass()) + RBDS->get_vel_cm();
    Matrix I = RBDS->get_moment_of_inertia();
    Vector vel_ang = I.inverse() * tau * dt + RBDS->get_vel_angular();

    RBDS->set_vel_cm(vel_cm);
    RBDS->set_vel_angular(vel_ang);
}


std::tuple<Vector, Vector> RBDSolverBase::totalForce_and_tau(ForcePtrContainer& forces, const RigidBodyState &RBDS)
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


void RBDLeapFrogSolver::updateRBDS(const double& dt, const RigidBodyState& RBDS, ForcePtrContainer& forces)
{
    // S_lf(dt) = S_x(dt/2)S_v(dt)S_x(dt/2)
    RBDLeapFrogSolver::updatePos(dt / 2, RBDS);
    RBDLeapFrogSolver::updateVel(dt, RBDS, forces);
    RBDLeapFrogSolver::updatePos(dt / 2, RBDS);
}


pba::RBDSolverPtr pba::CreateRBDLeapFrogSolver()
{
    return RBDSolverPtr(new RBDLeapFrogSolver);
}

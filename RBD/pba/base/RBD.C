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

void RBDSolverBase::updatePosWithCollision(const double &dt, const RigidBodyState &RBDS,  GeometryPtr geom)
{
    updatePos(dt, RBDS);
    RBDCollision::RBD_Collision(dt, RBDS, geom);
}

void RBDSolverBase::updateVel(const double &dt, const RigidBodyState& RBDS, ForcePtrContainer& forces)
{
    Vector total_force;
    Vector tau;
    std::tie(total_force, tau) = pba::totalForce_and_tau(forces, RBDS);
    Vector vel_cm = total_force * dt / (RBDS->get_total_mass()) + RBDS->get_vel_cm();
    Matrix I = RBDS->get_moment_of_inertia();
    Vector vel_ang = I.inverse() * tau * dt + RBDS->get_vel_angular();

    RBDS->set_vel_cm(vel_cm);
    RBDS->set_vel_angular(vel_ang);
}


//! ------------------------------------------------------------------------------------------------------------


void RBDLeapFrogSolver::updateRBDS(const double& dt, const RigidBodyState& RBDS, ForcePtrContainer& forces)
{
    // S_lf(dt) = S_x(dt/2)S_v(dt)S_x(dt/2)
    RBDLeapFrogSolver::updatePos(dt / 2, RBDS);
    RBDLeapFrogSolver::updateVel(dt, RBDS, forces);
    RBDLeapFrogSolver::updatePos(dt / 2, RBDS);
}

void RBDLeapFrogSolver::updateRBDSWithCollision(const double &dt, const RigidBodyState &RBDS,
                                                ForcePtrContainer &forces, GeometryPtr geom)
{
    RBDLeapFrogSolver::updatePosWithCollision(dt / 2, RBDS, geom);
    RBDLeapFrogSolver::updateVel(dt, RBDS, forces);
    RBDLeapFrogSolver::updatePosWithCollision(dt / 2, RBDS, geom);
}


pba::RBDSolverPtr pba::CreateRBDLeapFrogSolver()
{
    return RBDSolverPtr(new RBDLeapFrogSolver);
}


//! ------------------------------------------------------------------------------------------------------------

void RBDSubSolver::updateRBDSWithCollision(const double &dt, const RigidBodyState &RBDS, ForcePtrContainer &forces,
                                           GeometryPtr geom)
{
    assert(solver != nullptr);
    double sub_dt = dt / substep;
    for (int i = 0; i < substep; ++i) { solver->updateRBDSWithCollision(sub_dt, RBDS, forces, geom); }
}

pba::RBDSubSolverPtr pba::CreateRBDSubSolver()
{
    return RBDSubSolverPtr(new RBDSubSolver);
}

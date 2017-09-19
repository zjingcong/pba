
# include "Solver.h"
# include <iostream>

using namespace std;
using namespace pba;


//! ------------------------------------ SolverBase -------------------------------------------------

void SolverBase::updateSinglePos(const double& dt, DynamicalState DS, size_t i)
{
    Vector pos = DS->pos(i) + dt * DS->vel(i);
    DS->set_pos(i, pos);
}

void SolverBase::updatePos(const double& dt, DynamicalState DS)
{
    for (size_t i = 0; i < DS->nb(); ++i)
    {
        updateSinglePos(dt, DS, i);
    }
}

void SolverBase::updateVel(const double& dt, DynamicalState DS, ForcePtr force)
{
    for (size_t i = 0; i < DS->nb(); ++i)
    {
        float mass = DS->mass(i);
        Vector vel = DS->vel(i) + force->getForce(DS, i) * dt / mass;
        DS->set_vel(i, vel);
    }
}

void SolverBase::updatePosWithCollision(const double& dt, DynamicalState DS, CollisionPtr collision)
{
    // loop over particles
    for (size_t i = 0; i < DS->nb(); ++i)
    {
        // update position
        updateSinglePos(dt, DS, i);
        // collision detection and collision handling
        collision->collision(dt, DS, i);
    }
}


//! ------------------------------------ LeapFrog -------------------------------------------------

void LeapFrogSolver::_updateDS(const double& dt, DynamicalState DS, ForcePtr force)
{
    // S_lf(dt) = S_x(dt/2)S_v(dt)S_x(dt/2)
    LeapFrogSolver::updatePos(dt / 2, DS);
    LeapFrogSolver::updateVel(dt, DS, force);
    LeapFrogSolver::updatePos(dt / 2, DS);
}

void LeapFrogSolver::_updateDSWithCollision(const double& dt, DynamicalState DS, ForcePtr force, CollisionPtr collision)
{
    LeapFrogSolver::updatePosWithCollision(dt / 2, DS, collision);
    LeapFrogSolver::updateVel(dt, DS, force);
    LeapFrogSolver::updatePosWithCollision(dt / 2, DS, collision);
}


//! ------------------------------------ SixthOrder -------------------------------------------------

void SixOrderSolver::_updateDS(const double& dt, DynamicalState DS, ForcePtr force)
{
    double a = 1.0 / (4.0 - std::pow(4, 1.0/3.0));
    double b = 1 - 4 * a;
    double dt_a = a * dt;
    double dt_b = b * dt;

    LeapFrogSolver::_updateDS(dt_a, DS, force);
    LeapFrogSolver::_updateDS(dt_a, DS, force);
    LeapFrogSolver::_updateDS(dt_b, DS, force);
    LeapFrogSolver::_updateDS(dt_a, DS, force);
    LeapFrogSolver::_updateDS(dt_a, DS, force);
}

void SixOrderSolver::_updateDSWithCollision(const double& dt, DynamicalState DS, ForcePtr force, CollisionPtr collision)
{
    double a = 1.0 / (4.0 - std::pow(4, 1.0/3.0));
    double b = 1 - 4 * a;
    double dt_a = a * dt;
    double dt_b = b * dt;

    LeapFrogSolver::_updateDSWithCollision(dt_a, DS, force, collision);
    LeapFrogSolver::_updateDSWithCollision(dt_a, DS, force, collision);
    LeapFrogSolver::_updateDSWithCollision(dt_b, DS, force, collision);
    LeapFrogSolver::_updateDSWithCollision(dt_a, DS, force, collision);
    LeapFrogSolver::_updateDSWithCollision(dt_a, DS, force, collision);
}

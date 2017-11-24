
# include "Solver.h"
# include "omp.h"
# include <algorithm>

using namespace std;
using namespace pba;


//! ------------------------------------ SolverBase -------------------------------------------------

void SolverBase::updateDS(const double &dt, DynamicalState DS, ForcePtrContainer forces)
{
    _updateDS(dt, DS, forces);
    DS->update_time(dt);
}

void SolverBase::updateDSWithCollision(const double& dt, DynamicalState DS, ForcePtrContainer forces)
{
    _updateDSWithCollision(dt, DS, forces, false);
    DS->update_time(dt);
}

void SolverBase::updateDSWithCollisionWithKdTree(const double& dt, DynamicalState DS, ForcePtrContainer forces)
{
    _updateDSWithCollision(dt, DS, forces, true);
    DS->update_time(dt);
}

void SolverBase::updateSinglePos(const double& dt, DynamicalState DS, size_t i)
{
    Vector pos = DS->pos(i) + dt * DS->vel(i);
    DS->set_pos(i, pos);
}

void SolverBase::updatePos(const double& dt, DynamicalState DS)
{
    size_t i;
    for (i = 0; i < DS->nb(); ++i)
    {
        updateSinglePos(dt, DS, i);
    }
}

void SolverBase::updateVel(const double& dt, DynamicalState DS, ForcePtrContainer forces)
{
    size_t i;
    for (i = 0; i < DS->nb(); ++i)
    {
        float mass = DS->mass(i);
        Vector force_value = Vector(0.0, 0.0, 0.0);
        for (auto& it: forces)
        {
            force_value += it->getForce(i);
        }
        Vector vel = DS->vel(i) + force_value * dt / mass;
        DS->set_vel(i, vel);
    }
}

void SolverBase::updatePosWithCollision(const double& dt, DynamicalState DS, bool onkdTree)
{
    updatePos(dt, DS);
    if (onkdTree)   { collision->collisionWithKdTree(dt);}
    else    { collision->collision(dt);}
}


//! ------------------------------------ LeapFrog -------------------------------------------------

void LeapFrogSolver::_updateDS(const double& dt, DynamicalState DS, ForcePtrContainer forces)
{
    // S_lf(dt) = S_x(dt/2)S_v(dt)S_x(dt/2)
    LeapFrogSolver::updatePos(dt / 2, DS);
    LeapFrogSolver::updateVel(dt, DS, forces);
    LeapFrogSolver::updatePos(dt / 2, DS);
}

void LeapFrogSolver::_updateDSWithCollision(const double& dt, DynamicalState DS, ForcePtrContainer forces, bool onkdTree)
{
    LeapFrogSolver::updatePosWithCollision(dt / 2, DS, onkdTree);
    LeapFrogSolver::updateVel(dt, DS, forces);
    LeapFrogSolver::updatePosWithCollision(dt / 2, DS, onkdTree);
}


pba::SolverPtr pba::CreateLeapFrogSolver()
{
    return SolverPtr(new LeapFrogSolver);
}


//! ------------------------------------ SixthOrder -------------------------------------------------

void SixOrderSolver::_updateDS(const double& dt, DynamicalState DS, ForcePtrContainer forces)
{
    double a = 1.0 / (4.0 - std::pow(4, 1.0/3.0));
    double b = 1 - 4 * a;
    double dt_a = a * dt;
    double dt_b = b * dt;

    LeapFrogSolver::_updateDS(dt_a, DS, forces);
    LeapFrogSolver::_updateDS(dt_a, DS, forces);
    LeapFrogSolver::_updateDS(dt_b, DS, forces);
    LeapFrogSolver::_updateDS(dt_a, DS, forces);
    LeapFrogSolver::_updateDS(dt_a, DS, forces);
}

void SixOrderSolver::_updateDSWithCollision(const double& dt, DynamicalState DS, ForcePtrContainer forces, bool onkdTree)
{
    double a = 1.0 / (4.0 - std::pow(4, 1.0/3.0));
    double b = 1 - 4 * a;
    double dt_a = a * dt;
    double dt_b = b * dt;

    LeapFrogSolver::_updateDSWithCollision(dt_a, DS, forces, onkdTree);
    LeapFrogSolver::_updateDSWithCollision(dt_a, DS, forces, onkdTree);
    LeapFrogSolver::_updateDSWithCollision(dt_b, DS, forces, onkdTree);
    LeapFrogSolver::_updateDSWithCollision(dt_a, DS, forces, onkdTree);
    LeapFrogSolver::_updateDSWithCollision(dt_a, DS, forces, onkdTree);
}


pba::SolverPtr pba::CreateSixOrderSolver()
{
    return SolverPtr(new SixOrderSolver);
}

/// ------------------------------------ Subsolver ---------------------------------------------------

void SubSolver::updateDSWithCollision(const double& dt, DynamicalState DS, ForcePtrContainer forces)
{
    assert(solver != nullptr);
    double sub_dt = dt / substep;
    for (int i = 0; i < substep; ++i) { solver->updateDSWithCollision(sub_dt, DS, forces); }
}

void SubSolver::updateDSWithCollisionWithKdtree(const double &dt, DynamicalState DS, ForcePtrContainer forces)
{
    assert(solver != nullptr);
    double sub_dt = dt / substep;
    for (int i = 0; i < substep; ++i) { solver->updateDSWithCollisionWithKdTree(sub_dt, DS, forces); }
}


pba::SubSolverPtr pba::CreateSubSolver()
{
    return SubSolverPtr(new SubSolver);
}

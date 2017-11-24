
# include "Solver.h"
# include "omp.h"
# include <algorithm>

using namespace std;
using namespace pba;


//! ------------------------------------ SolverBase -------------------------------------------------

void SolverBase::updateDSWithCollision(const double& dt, DynamicalState DS, ForcePtrContainer forces, GeometryPtr geom, const double& Cr, const double& Cs)
{
    _updateDSWithCollision(dt, DS, forces, geom, Cr, Cs, false);
    DS->update_time(dt);
}

void SolverBase::updateDSWithCollisionWithKdTree(const double& dt, DynamicalState DS, ForcePtrContainer forces, GeometryPtr geom, const double& Cr, const double& Cs)
{
    _updateDSWithCollision(dt, DS, forces, geom, Cr, Cs, true);
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
            force_value += it->getForce(DS, i);
        }
        Vector vel = DS->vel(i) + force_value * dt / mass;
        DS->set_vel(i, vel);
    }
}

void SolverBase::updatePosWithCollision(const double& dt, DynamicalState DS, GeometryPtr geom, const double& Cr, const double& Cs, bool onkdTree)
{
    updatePos(dt, DS);
    if (onkdTree)   {TriangleCollision::triangleCollisionWithKdTree(dt, DS, geom, Cr, Cs);}
    else    {TriangleCollision::triangleCollision(dt, DS, geom, Cr, Cs);}
}


//! ------------------------------------ LeapFrog -------------------------------------------------

void LeapFrogSolver::_updateDS(const double& dt, DynamicalState DS, ForcePtrContainer forces)
{
    // S_lf(dt) = S_x(dt/2)S_v(dt)S_x(dt/2)
    LeapFrogSolver::updatePos(dt / 2, DS);
    LeapFrogSolver::updateVel(dt, DS, forces);
    LeapFrogSolver::updatePos(dt / 2, DS);
}

void LeapFrogSolver::_updateDSWithCollision(const double& dt, DynamicalState DS, ForcePtrContainer forces, GeometryPtr geom, const double& Cr, const double& Cs, bool onkdTree)
{
    LeapFrogSolver::updatePosWithCollision(dt / 2, DS, geom, Cr, Cs, onkdTree);
    LeapFrogSolver::updateVel(dt, DS, forces);
    LeapFrogSolver::updatePosWithCollision(dt / 2, DS, geom, Cr, Cs, onkdTree);
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

void SixOrderSolver::_updateDSWithCollision(const double& dt, DynamicalState DS, ForcePtrContainer forces, GeometryPtr geom, const double& Cr, const double& Cs, bool onkdTree)
{
    double a = 1.0 / (4.0 - std::pow(4, 1.0/3.0));
    double b = 1 - 4 * a;
    double dt_a = a * dt;
    double dt_b = b * dt;

    LeapFrogSolver::_updateDSWithCollision(dt_a, DS, forces, geom, Cr, Cs, onkdTree);
    LeapFrogSolver::_updateDSWithCollision(dt_a, DS, forces, geom, Cr, Cs, onkdTree);
    LeapFrogSolver::_updateDSWithCollision(dt_b, DS, forces, geom, Cr, Cs, onkdTree);
    LeapFrogSolver::_updateDSWithCollision(dt_a, DS, forces, geom, Cr, Cs, onkdTree);
    LeapFrogSolver::_updateDSWithCollision(dt_a, DS, forces, geom, Cr, Cs, onkdTree);
}


pba::SolverPtr pba::CreateSixOrderSolver()
{
    return SolverPtr(new SixOrderSolver);
}


# include "Solver.h"
# include <iostream>

using namespace std;
using namespace pba;


//! ------------------------------------ SolverBase -------------------------------------------------

void SolverBase::updateSinglePos(double dt, size_t i)
{
    Vector pos = DS->pos(i) + dt * DS->vel(i);
    DS->set_pos(i, pos);
}

void SolverBase::updateSingleVel(double dt, size_t i)
{
    float mass = DS->mass(i);
    Vector vel = DS->vel(i) + force->getForce(DS, i) * dt / mass;
    DS->set_vel(i, vel);
}

void SolverBase::updatePos(double dt)
{
    for (size_t i = 0; i < DS->nb(); ++i)
    {
        updateSinglePos(dt, i);
    }
}

void SolverBase::updateVel(double dt)
{
    for (size_t i = 0; i < DS->nb(); ++i)
    {
        updateSingleVel(dt, i);
    }
}

void SolverBase::updatePosWithCollision(double dt)
{
    CollisionPtr collision = new TriangleCollision(DS, geom, dt); // there's a geometry, need collision detection
    // loop over particles
    for (size_t i = 0; i < DS->nb(); ++i)
    {
        // update position
        updateSinglePos(dt, i);
        if (!geom.empty())
        {
            // collision detection and collision handling
            collision->collision(i, Cr, Cs);
        }
    }
}


//! ------------------------------------ LeapFrog -------------------------------------------------

void LeapFrogSolver::_updateDS(double dt)
{
    // S_lf(dt) = S_x(dt/2)S_v(dt)S_x(dt/2)
    LeapFrogSolver::updatePos(dt / 2);
    LeapFrogSolver::updateVel(dt);
    LeapFrogSolver::updatePos(dt / 2);
}


//! ------------------------------------ SixthOrder -------------------------------------------------

void SixOrderSolver::_updateDS(double dt)
{
    double a = 1.0 / (4.0 - std::pow(4, 1.0/3.0));
    double b = 1 - 4 * a;
    double dt_a = a * dt;
    double dt_b = b * dt;

    LeapFrogSolver::updateDS(dt_a);
    LeapFrogSolver::updateDS(dt_a);
    LeapFrogSolver::updateDS(dt_b);
    LeapFrogSolver::updateDS(dt_a);
    LeapFrogSolver::updateDS(dt_a);
}

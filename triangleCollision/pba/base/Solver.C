
# include "Solver.h"
# include <iostream>

using namespace std;
using namespace pba;

void SolverBase::updatePos(double dt)
{
    for (size_t i = 0; i < DS->nb(); ++i)
    {
        Vector pos = DS->pos(i) + dt * DS->vel(i);
        DS->set_pos(i, pos);
    }
}

void SolverBase::updateVel(double dt)
{
    for (size_t i = 0; i < DS->nb(); ++i)
    {
        float mass = DS->mass(i);
        Vector vel = DS->vel(i) + force->getForce(DS, i) * dt / mass;
        DS->set_vel(i, vel);
    }
}


void LeapFrogSolver::_updateDS(double dt)
{
    // S_lf(dt) = S_x(dt/2)S_v(dt)S_x(dt/2)
    LeapFrogSolver::updatePos(dt / 2);
    LeapFrogSolver::updateVel(dt);
    LeapFrogSolver::updatePos(dt / 2);
}


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

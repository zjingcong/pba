
# include "Solver.h"
# include <iostream>
# include <algorithm>

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

void SolverBase::updatePosWithCollision(const double& dt, DynamicalState DS, GeometryPtr geom, const double& Cr, const double& Cs)
{
    // loop over particles
    for (size_t i = 0; i < DS->nb(); ++i)
    {
        // loop until no collisions
        bool collision_flag = true;
        while(collision_flag)
        {
            // update position
            updateSinglePos(dt, DS, i);

            // loop over triangles
            int collision_num = 0;
            std::vector<double> tmp_delta_times;
            std::vector<Vector> tmp_collision_positions;
            std::vector<size_t> tmp_collision_indices;
            for (size_t p = 0; p < geom->get_nb(); ++p)
            {
                // collision detection
                TrianglePtr triangle = geom->get_triangles()[p];
                double dti;
                Vector xi;
                if (TriangleCollision::collisionDetection(dt, DS, i, triangle, dti, xi))
                {
                    collision_num++;
                    tmp_delta_times.push_back(dti);
                    tmp_collision_positions.push_back(xi);
                    tmp_collision_indices.push_back(p);
                }
            }
            if (collision_num == 0)   {collision_flag = false;} // no collision

            if (collision_flag)
            {
                // pick the maximum dt_i
                auto max = std::max_element(tmp_delta_times.begin(), tmp_delta_times.end());
                double dt_i = *max;
                long max_index = std::distance(tmp_delta_times.begin(), max);
                Vector x_i = tmp_collision_positions[max_index];
                size_t collision_triangle_index = tmp_collision_indices[max_index];
                // handle collision for maximum dt_i
                TriangleCollision::collisionHandling(dt, DS, i, geom->get_triangles().at(collision_triangle_index), dt_i, x_i, Cr, Cs);
                // store collision triangle
                geom->add_collisions(collision_triangle_index);
            }
        }
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

void LeapFrogSolver::_updateDSWithCollision(const double& dt, DynamicalState DS, ForcePtr force, GeometryPtr geom, const double& Cr, const double& Cs)
{
    LeapFrogSolver::updatePosWithCollision(dt / 2, DS, geom, Cr, Cs);
    LeapFrogSolver::updateVel(dt, DS, force);
    LeapFrogSolver::updatePosWithCollision(dt / 2, DS, geom, Cr, Cs);
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

void SixOrderSolver::_updateDSWithCollision(const double& dt, DynamicalState DS, ForcePtr force, GeometryPtr geom, const double& Cr, const double& Cs)
{
    double a = 1.0 / (4.0 - std::pow(4, 1.0/3.0));
    double b = 1 - 4 * a;
    double dt_a = a * dt;
    double dt_b = b * dt;

    LeapFrogSolver::_updateDSWithCollision(dt_a, DS, force, geom, Cr, Cs);
    LeapFrogSolver::_updateDSWithCollision(dt_a, DS, force, geom, Cr, Cs);
    LeapFrogSolver::_updateDSWithCollision(dt_b, DS, force, geom, Cr, Cs);
    LeapFrogSolver::_updateDSWithCollision(dt_a, DS, force, geom, Cr, Cs);
    LeapFrogSolver::_updateDSWithCollision(dt_a, DS, force, geom, Cr, Cs);
}

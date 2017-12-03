//
// Created by jingcoz on 12/2/17.
//

# include "PbaHou.h"
# include <iostream>

namespace pba
{

    PbaHouParticles::~PbaHouParticles() {}
    void PbaHouParticles::init(const std::vector<std::string>& args)
    {
        std::cout << "pba houdini init" << std::endl;

    }

    void PbaHouParticles::solve()
    {
        solver->updateDS(dt, DS, forces);
    }

    void PbaHouParticles::add_DS(int num)
    {
        assert(num >= 0);
        DS->add(size_t(num));
    }

    void PbaHouParticles::set_pos(int p, double x, double y, double z)
    {
        assert(p <= DS->nb() && p >= 0);
        Vector pos = Vector(x, y, z);
        DS->set_pos(size_t(p), pos);
    }

    void PbaHouParticles::set_vel(int p, double x, double y, double z)
    {
        assert(p <= DS->nb() && p >= 0);
        Vector v = Vector(x, y, z);
        DS->set_vel(size_t(p), v);
    }

    void PbaHouParticles::set_dt(double delta_t)
    {
        dt = delta_t;
    }

    void PbaHouParticles::set_gravity(float g)
    {
        gravity->update_parms("g", g);
    }

    double* PbaHouParticles::get_pos(int p)
    {
        assert(p <= DS->nb() && p >= 0);
        Vector pos = DS->pos(size_t(p));
        double* pPos = new double[3];
        pPos[0] = pos.X();
        pPos[1] = pos.Y();
        pPos[2] = pos.Z();

        return pPos;
    }

    int PbaHouParticles::get_nb()
    {
        return int(DS->nb());
    }

    void PbaHouParticles::reset()
    {
        DS->clear();
    }

    PbaHouParticles* PbaHouParticles::pPbaHouParticles = nullptr;

    PbaHouParticles::PbaHouParticles():
            dt(1.0/24.0)
    {
        DS = CreateDynamicalState("ParticlesDynamic");
        solver = CreateLeapFrogSolver();
        gravity = CreateGravity(DS, 0.98); // add gravity
        forces.push_back(gravity);
    }

    PbaHouParticles* CreatePbaHouParticles()  { return PbaHouParticles::Instance();}

}

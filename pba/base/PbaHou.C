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
        std::cout << "Pba Houdini Init" << std::endl;
        DS->create_attr("life", float(100.0));
        DS->create_attr("age", float(-1.0));
        DS->create_attr("pscale", float(1.0));
        DS->create_attr("dead", int(false));
    }

    void PbaHouParticles::solve()
    {
        solver->updateDS(dt, DS, forces);
        current_frame++;
        // increase age
        for (size_t i = 0; i < DS->nb(); ++i)
        {
            float age = DS->get_float_attr("age", i);
            DS->set_attr("age", i, float(age + 1.0));
        }
        // set dead
        for (size_t i = 0; i < DS->nb(); ++i)
        {
            float life = DS->get_float_attr("life", i);
            float age = DS->get_float_attr("age", i);
            bool dead_condition = (age > life);
            DS->set_attr("dead", i, int(dead_condition));
        }
    }

    void PbaHouParticles::clean()
    {
        for (size_t i = 0, delete_num = 0; i < DS->nb(); ++i)
        {
            int dead = DS->get_int_attr("dead", i - delete_num);
            if (dead)
            {
                DS->remove(i - delete_num);
                delete_num++;
            }
        }
    }

    void PbaHouParticles::add_DS(int num, float life, float variance)
    {
        assert(num >= 0);
        size_t nb = DS->nb();
        DS->add(size_t(num));
        // auto assign id based on the generation
        for (size_t i = 0; i < num; ++i)
        {
            size_t id = nb + i;
            DS->set_id(id, int(id));
            float l = float(life + 2 * variance * (drand48() - 0.5));
            DS->set_attr("life", id, l);
        }
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

    void PbaHouParticles::set_current_frame(int f)
    {
        current_frame = f;
    }

    double* PbaHouParticles::get_pos(int p)
    {
        assert(p <= int(DS->nb()) && p >= 0);
        Vector pos = DS->pos(size_t(p));
        double* pPos = new double[3];
        pPos[0] = pos.X();
        pPos[1] = pos.Y();
        pPos[2] = pos.Z();

        return pPos;
    }

    double* PbaHouParticles::get_vel(int p)
    {
        assert(p <= int(DS->nb()) && p >= 0);
        Vector vel = DS->vel(size_t(p));
        double* pVel = new double[3];
        pVel[0] = vel.X();
        pVel[1] = vel.Y();
        pVel[2] = vel.Z();

        return pVel;
    }

    int PbaHouParticles::get_id(int p)
    {
        assert(p <= int(DS->nb()) && p >= 0);
        return DS->id(size_t(p));
    }

    int PbaHouParticles::get_nb()
    {
        return int(DS->nb());
    }

    float PbaHouParticles::get_age(int p)
    {
        return DS->get_float_attr("age", size_t(p));
    }

    float PbaHouParticles::get_life(int p)
    {
        return DS->get_float_attr("life", size_t(p));
    }

    float PbaHouParticles::get_pscale(int p)
    {
        return DS->get_float_attr("pscale", size_t(p));
    }

    int PbaHouParticles::get_dead(int p)
    {
        return DS->get_int_attr("dead", size_t(p));
    }

    void PbaHouParticles::reset()
    {
        DS->clear();
    }

    PbaHouParticles* PbaHouParticles::pPbaHouParticles = nullptr;

    PbaHouParticles::PbaHouParticles():
            dt(1.0/24.0),
            current_frame(0)
    {
        DS = CreateDynamicalState("ParticlesDynamic");
        solver = CreateLeapFrogSolver();
        gravity = CreateGravity(DS, 0.98); // add gravity
        forces.push_back(gravity);
    }

    PbaHouParticles* CreatePbaHouParticles()  { return PbaHouParticles::Instance();}

}

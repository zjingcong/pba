//
// Created by jingcoz on 12/2/17.
//

# include "PbaHou.h"
# include <iostream>

namespace pba
{

    PbaHouParticles::~PbaHouParticles()
    {
        delete perlin_noise;
    }

    void PbaHouParticles::init(const std::vector<std::string>& args)
    {
        std::cout << "Pba Houdini Init" << std::endl;
        DS->create_attr("life", float(100.0));
        DS->create_attr("birth_life", float(100.0));
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

    void PbaHouParticles::add_DS(int num, float life, float variance, float pscale)
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
            DS->set_attr("birth_life", id, l);
            float scale = float(pscale * 0.5 * (drand48() + 1.0));
            DS->set_attr("pscale", id, scale);
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

    void PbaHouParticles::update_status(double vel_max)
    {
        for (size_t i = 0; i < DS->nb(); ++i)
        {
            Vector vel = DS->vel(i);
            double vel_mag = vel.magnitude();
            double f = vel_mag / vel_max;

            double color_factor = clamp(f, 0.0, 1.0);
            Color cd = Color(1.0, float(color_factor), 0.0, 1.0);
            DS->set_ci(i, cd);

            double life_factor = f * 0.5 + 0.5;
            float life = DS->get_float_attr("birth_life", i) * float(life_factor);
            DS->set_attr("life", i, life);
        }
    }

    void PbaHouParticles::create_noise(float gamma, float freq, float f_jump, float octaves, float offs)
    {
        Noise_t parms;
        parms.gamma = gamma;
        parms.frequency = freq;
        parms.fjump = f_jump;
        parms.octaves = octaves;

        noise_offset = offs;

        std::cout << "create perlin noise" << std::endl;
        perlin_noise->setParameters(parms);
    }

    void PbaHouParticles::advect_by_noise(double scale)
    {
        for (size_t i = 0; i < DS->nb(); ++i)
        {
            Vector pos = DS->pos(i);
            Vector noise_vec = noise_field(pos);
            DS->set_vel(i, scale * noise_vec);
        }
    }

    void PbaHouParticles::add_by_noise(double scale)
    {
        for (size_t i = 0; i < DS->nb(); ++i)
        {
            Vector pos = DS->pos(i);
            Vector noise_vec = noise_field(pos);
            Vector vel = DS->vel(i);
            DS->set_vel(i, scale * noise_vec + vel);
        }
    }

    void PbaHouParticles::set_pos(int p, double x, double y, double z)
    {
        assert(p <= int(DS->nb()) && p >= 0);
        Vector pos = Vector(x, y, z);
        DS->set_pos(size_t(p), pos);
    }

    void PbaHouParticles::set_vel(int p, double x, double y, double z)
    {
        assert(p <= int(DS->nb()) && p >= 0);
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

    double* PbaHouParticles::get_cd(int p)
    {
        assert(p <= int(DS->nb()) && p >= 0);
        Color cd = DS->ci(size_t(p));
        double* pCd = new double[4];
        pCd[0] = cd.red();
        pCd[1] = cd.green();
        pCd[2] = cd.blue();
        pCd[3] = cd.alpha();

        return pCd;
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

    double PbaHouParticles::clamp(double value, double min, double max)
    {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    Vector PbaHouParticles::noise_field(const Vector &pos)
    {
        Noise_t noise_parms = perlin_noise->getNoiseParameters();
        double scale = pow( 1.0 + noise_parms.roughness, noise_parms.octaves - 1.0);
        Vector delta_x = noise_offset * Vector(1.0, 1.0, 1.0);
        Vector delta_y = noise_offset * Vector(1.0, 0.0, -1.0);
        Vector delta_z = noise_offset * Vector(0.0, 1.0, 0.0);

        double n0 = (2 * perlin_noise->eval(pos)) / scale;
        double nx = (2 * perlin_noise->eval(pos + delta_x)) / scale;
        double ny = (2 * perlin_noise->eval(pos + delta_y)) / scale;
        double nz = (2 * perlin_noise->eval(pos + delta_z)) / scale;

        Vector vec = Vector(nx - n0, ny - n0, nz - n0);

        return vec;
    }


    PbaHouParticles* PbaHouParticles::pPbaHouParticles = nullptr;

    PbaHouParticles::PbaHouParticles():
            dt(1.0/24.0),
            current_frame(0),
            noise_offset(0.005),
            life(1000.0),
            variance(0.0)
    {
        DS = CreateDynamicalState("ParticlesDynamic");
        solver = CreateLeapFrogSolver();
        gravity = CreateGravity(DS, 0.98); // add gravity
        forces.push_back(gravity);
        perlin_noise = new FractalSum<PerlinNoiseGustavson>;
    }

    PbaHouParticles* CreatePbaHouParticles()  { return PbaHouParticles::Instance();}

}

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
    }

    void PbaHouParticles::solve()
    {
        solver->updateDS(dt, DS, forces);
        current_frame++;
    }

    void PbaHouParticles::advect(std::string volume_path)
    {
        std::cout << "Load velocity field..." << std::endl;
        Vec3fGrid::Ptr vel_grid = readVDBGrid<Vec3fTree>(volume_path, "vel");
        AdvectByVolume::advect_by_volume(DS, vel_grid);

        // test
        AdvectByVolume::loop_grid(vel_grid);
    }

    void PbaHouParticles::test(double x, double y, double z, std::string volume_path)
    {
//        std::cout << "Load velocity field..." << std::endl;
//        Vec3fGrid::Ptr vel_grid = readVDBGrid<Vec3fTree>(volume_path, "vel");
//        Vector vel = AdvectByVolume::get_grid_value(Vector(x, y, z), vel_grid);
//
//        AdvectByVolume::loop_grid(vel_grid);
//
//        vel.printValue();
//        std::cout << std::endl;
        FloatGrid::Ptr float_grid = readVDBGrid<FloatTree>(volume_path, "tst");
        AdvectByVolume::loop_floatgrid(float_grid);
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

    void PbaHouParticles::set_current_frame(int f)
    {
        current_frame = f;
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

    double* PbaHouParticles::get_vel(int p)
    {
        assert(p <= DS->nb() && p >= 0);
        Vector vel = DS->vel(size_t(p));
        double* pVel = new double[3];
        pVel[0] = vel.X();
        pVel[1] = vel.Y();
        pVel[2] = vel.Z();

        return pVel;
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

//
// Created by jingcoz on 10/20/17.
//

#ifndef PBA_RBD_H
#define PBA_RBD_H

# include <string>
# include "RigidBodyState.h"
# include "LinearAlgebra.h"
# include "Force.h"
# include "Collision.h"
# include <tuple>

namespace pba
{

    class RBDSolverBase
    {
    public:
        RBDSolverBase(): name("RBDSolverUnknown") {}
        virtual ~RBDSolverBase()    {}

        virtual void updateRBDS(const double& dt, const RigidBodyState& RBDS, ForcePtrContainer& forces)    {}
        virtual void updateRBDSWithCollision(const double& dt, const RigidBodyState& RBDS, ForcePtrContainer& forces, GeometryPtr geom)   {}

    protected:
        std::string name;
        void updatePos(const double& dt, const RigidBodyState& RBDS);
        void updatePosWithCollision(const double& dt, const RigidBodyState& RBDS, GeometryPtr geom);
        void updateVel(const double& dt, const RigidBodyState& RBDS, ForcePtrContainer& forces);
    };

    typedef std::shared_ptr<RBDSolverBase> RBDSolverPtr;

    class RBDLeapFrogSolver: public RBDSolverBase
    {
    public:
        RBDLeapFrogSolver() {name = "RBDLeapFrog";}
        void updateRBDS(const double& dt, const RigidBodyState& RBDS, ForcePtrContainer& forces);
        void updateRBDSWithCollision(const double& dt, const RigidBodyState& RBDS, ForcePtrContainer& forces, GeometryPtr geom);
    };

    RBDSolverPtr CreateRBDLeapFrogSolver();


    class RBDSubSolver
    {
    public:
        RBDSubSolver(): solver(nullptr), substep(1)  {}
        ~RBDSubSolver() {}

        void setSolver(const RBDSolverPtr& s) {solver = s;}
        void setSubstep(const int& sub_step)   {substep = sub_step;}

        void updateRBDSWithCollision(const double& dt, const RigidBodyState& RBDS, ForcePtrContainer& forces, GeometryPtr geom);

    private:
        RBDSolverPtr solver;
        int substep;
    };

    typedef std::shared_ptr<RBDSubSolver> RBDSubSolverPtr;
    RBDSubSolverPtr CreateRBDSubSolver();

}

#endif //PBA_RBD_H

//
// Created by jingcoz on 10/20/17.
//

#ifndef PBA_RBD_H
#define PBA_RBD_H

# include <string>
# include "RigidBodyState.h"
# include "LinearAlgebra.h"
# include "Force.h"
# include <tuple>

namespace pba
{

    class RBDSolverBase
    {
    public:
        RBDSolverBase(): name("RBDSolverUnknown") {}
        virtual ~RBDSolverBase()    {}

        virtual void updateRBDS(const double& dt, const RigidBodyState& RBDS, ForcePtrContainer& forces)    {}

    protected:
        std::string name;
        void updatePos(const double& dt, const RigidBodyState& RBDS);
        void updateVel(const double& dt, const RigidBodyState& RBDS, ForcePtrContainer& forces);

        std::tuple<Vector, Vector> totalForce_and_tau(ForcePtrContainer& forces, const RigidBodyState &RBDS);
    };

    typedef std::shared_ptr<RBDSolverBase> RBDSolverPtr;

    class RBDLeapFrogSolver: public RBDSolverBase
    {
    public:
        RBDLeapFrogSolver() {name = "RBDLeapFrog";}
        void updateRBDS(const double& dt, const RigidBodyState& RBDS, ForcePtrContainer& forces);
    };

    RBDSolverPtr CreateRBDLeapFrogSolver();

}

#endif //PBA_RBD_H

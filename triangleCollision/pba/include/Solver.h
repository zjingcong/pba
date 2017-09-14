//
// Created by jingcoz on 9/13/17.
//

#ifndef PBA_SOLVER_H
#define PBA_SOLVER_H

# include <string>
# include "DynamicalState.h"
# include "Force.h"

namespace pba
{

    class SolverBase
    {
    public:
        SolverBase(): name("unknown")   {}
        virtual ~SolverBase()   {}

        const std::string& Name() const { return name; }
        virtual void updateDS(float dt) {}

        virtual void setForce(ForcePtr f)   {force = f;}
        virtual void setDS(DynamicalState ds)   {DS = ds;}

    protected:
        // partial solver: update position
        void updatePos(float dt);
        // partial solver: update velocity
        void updateVel(float dt);

        std::string name;
        ForcePtr force;
        DynamicalState DS;
    };


    class LeapFrogSolver: public SolverBase
    {
    public:
        LeapFrogSolver(): name("LeapFrog")  {}
        ~LeapFrogSolver()   {}

        void updateDS(float dt);
    };


    class SixOrderSolver: public LeapFrogSolver
    {
    public:
        SixOrderSolver(): name("SixOrder")  {}
        ~SixOrderSolver()   {}

        void updateDS(float dt);
    };

}

#endif //PBA_FORCE_H
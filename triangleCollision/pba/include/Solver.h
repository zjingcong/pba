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
        SolverBase(): name("partialSolver")   {}
        virtual ~SolverBase()   {}

        const std::string& Name() const { return name; }
        void updateDS(float dt) { _updateDS(dt); DS->update_time(dt);}

        virtual void setForce(ForcePtr f)   {force = f;}
        virtual void setDS(DynamicalState ds)   {DS = ds;}

    protected:
        // partial solver: update position
        void updatePos(float dt);
        // partial solver: update velocity
        void updateVel(float dt);

        virtual void _updateDS(float dt) {}

        std::string name;
        ForcePtr force;
        DynamicalState DS;
    };

    typedef SolverBase* SolverPtr;


    class LeapFrogSolver: public SolverBase
    {
    public:
        LeapFrogSolver()    {name="LeapFrog";}
        ~LeapFrogSolver()   {}

    protected:
        void _updateDS(float dt);
    };


    class SixOrderSolver: public LeapFrogSolver
    {
    public:
        SixOrderSolver()    {name="SixOrder";}
        ~SixOrderSolver()   {}

    protected:
        void _updateDS(float dt);
    };

}

#endif //PBA_FORCE_H
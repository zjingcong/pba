//
// Created by jingcoz on 9/13/17.
//

#ifndef PBA_SOLVER_H
#define PBA_SOLVER_H

# include <string>
# include "DynamicalState.h"
# include "Force.h"
# include "Collision.h"
# include "Geometry.h"

namespace pba
{

    class SolverBase
    {
    public:
        SolverBase():
                name("partialSolver"),
                // default collision: elastic collision
                Cr(1.0),
                Cs(1.0)   {}
        virtual ~SolverBase()   {}

        //! get solver name
        const std::string& Name() const { return name; }

        //! set forces
        void setForce(ForcePtr f)   {force = f;}
        //! set dynamical state
        void setDS(DynamicalState ds)   {DS = ds;}
        //! set geometry (triangles)
        void setGeom(Geom& geometry)    {geom = geometry;}
        //! set collision coefficients, default is elastic collision
        void setCollisionCoefficient(double cr, double cs)  {Cr = cr; Cs = cs;}

        //! partial solver: update position
        void updatePos(double dt);
        //! partial solver: update velocity
        void updateVel(double dt);
        //! partial solver: update position with collision
        void updatePosWithCollision(double dt);

        //! update vel and pos in DS, update time in DS
        void updateDS(double dt) { _updateDS(dt); DS->update_time(dt);}

    protected:
        virtual void _updateDS(double dt) {}

        std::string name;
        ForcePtr force;
        DynamicalState DS;
        Geom geom;
        double Cr;
        double Cs;

    private:
        void updateSinglePos(double dt, size_t i);
        void updateSingleVel(double dt, size_t i);
    };

    typedef SolverBase* SolverPtr;


    class LeapFrogSolver: public SolverBase
    {
    public:
        LeapFrogSolver()    {name="LeapFrog";}
        ~LeapFrogSolver()   {}

    protected:
        void _updateDS(double dt);
    };


    class SixOrderSolver: public LeapFrogSolver
    {
    public:
        SixOrderSolver()    {name="SixOrder";}
        ~SixOrderSolver()   {}

    protected:
        void _updateDS(double dt);
    };

}

#endif //PBA_FORCE_H
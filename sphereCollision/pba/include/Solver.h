//
// Created by jingcoz on 9/13/17.
//

#ifndef PBA_SOLVER_H
#define PBA_SOLVER_H

# include <string>
# include <iostream>
# include <cassert>
# include "DynamicalState.h"
# include "Force.h"
# include "Collision.h"
# include "Geometry.h"

namespace pba
{

//! ------------------------------------ SolverBase -------------------------------------------------

    class SolverBase
    {
    public:
        SolverBase(): name("unknown") {}
        virtual ~SolverBase()   {}

        //! get solver name
        const std::string& Name() const { return name; }

        //! set collision
        void setCollision(CollisionPtr& coll)    {collision = coll;}

        //! update vel and pos in DS, update time in DS
        void updateDS(const double& dt, DynamicalState& DS, ForcePtrContainer forces) { _updateDS(dt, DS, forces); DS->update_time(dt);}
        //! update vel and pos in DS with collision, update time in DS
        void updateDSWithCollision(const double& dt, DynamicalState& DS, ForcePtrContainer forces, GeometryPtr geom, const double& Cr, const double& Cs);
        //! update vel and pos in DS with collision, update time in DS, using KdTree for the geometry
        void updateDSWithCollisionWithKdTree(const double& dt, DynamicalState& DS, ForcePtrContainer forces, GeometryPtr geom, const double& Cr, const double& Cs);

    protected:
        virtual void _updateDS(const double& dt, DynamicalState& DS, ForcePtrContainer forces) {}
        virtual void _updateDSWithCollision(const double& dt, DynamicalState& DS, ForcePtrContainer forces, GeometryPtr geom, const double& Cr, const double& Cs, bool onkdTree)   {}

        //! partial solver: update position
        void updatePos(const double& dt, DynamicalState& DS);
        //! partial solver: update velocity
        void updateVel(const double& dt, DynamicalState& DS, ForcePtrContainer forces);
        //! partial solver: update position with collision
        void updatePosWithCollision(const double& dt, DynamicalState& DS, GeometryPtr geom, const double& Cr, const double& Cs, bool onkdTree);

        std::string name;
        CollisionPtr collision;

    private:
        void updateSinglePos(const double& dt, DynamicalState& DS, size_t i);
    };

    typedef std::shared_ptr<SolverBase> SolverPtr;


//! ------------------------------------ LeapFrog -------------------------------------------------

    class LeapFrogSolver: public SolverBase
    {
    public:
        LeapFrogSolver()    {name="LeapFrog";}
        ~LeapFrogSolver()   {}

    protected:
        void _updateDS(const double& dt, DynamicalState& DS, ForcePtrContainer forces);
        void _updateDSWithCollision(const double& dt, DynamicalState& DS, ForcePtrContainer forces, GeometryPtr geom, const double& Cr, const double& Cs, bool onkdTree);
    };

    SolverPtr CreateLeapFrogSolver();

//! ------------------------------------ SixthOrder -------------------------------------------------

    class SixOrderSolver: public LeapFrogSolver
    {
    public:
        SixOrderSolver()    {name="SixOrder";}
        ~SixOrderSolver()   {}

    protected:
        void _updateDS(const double& dt, DynamicalState& DS, ForcePtrContainer forces);
        void _updateDSWithCollision(const double& dt, DynamicalState& DS, ForcePtrContainer forces, GeometryPtr geom, const double& Cr, const double& Cs, bool onkdTree);
    };

    SolverPtr CreateSixOrderSolver();


/// ------------------------------------ Subsolver ---------------------------------------------------

    class SubSolver
    {
    public:
        SubSolver(): solver(nullptr), substep(1) {}
        ~SubSolver()    {}

        void setSolver(const SolverPtr& s) {solver = s;}
        void setSubstep(const int& sub_step)   {substep = sub_step;}

        void updateDSWithCollision(const double& dt, DynamicalState DS, ForcePtrContainer forces, GeometryPtr geom, const double& Cr, const double& Cs);
        void updateDSWithCollisionWithKdtree(const double& dt, DynamicalState DS, ForcePtrContainer forces, GeometryPtr geom, const double& Cr, const double& Cs);

    private:
        SolverPtr solver;
        int substep;
    };

    typedef std::shared_ptr<SubSolver> SubSolverPtr;
    SubSolverPtr CreateSubSolver();

}

#endif //PBA_FORCE_H
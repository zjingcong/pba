//
// Created by jingcoz on 9/13/17.
//

#ifndef PBA_SOLVER_H
#define PBA_SOLVER_H

# include <string>
# include <iostream>
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

        //! update vel and pos in DS, update time in DS
        void updateDS(const double& dt, DynamicalState DS, ForcePtr force) { _updateDS(dt, DS, force); DS->update_time(dt);}
        //! update vel and pos in DS with collision, update time in DS
        void updateDSWithCollision(const double& dt, DynamicalState DS, ForcePtr force, GeometryPtr geom, const double& Cr, const double& Cs)
        {
            geom->clean_collisions();
            _updateDSWithCollision(dt, DS, force, geom, Cr, Cs);
            DS->update_time(dt);
        }

    protected:
        virtual void _updateDS(const double& dt, DynamicalState DS, ForcePtr force) {}
        virtual void _updateDSWithCollision(const double& dt, DynamicalState DS, ForcePtr force, GeometryPtr geom, const double& Cr, const double& Cs)   {}

        //! partial solver: update position
        void updatePos(const double& dt, DynamicalState DS);
        //! partial solver: update velocity
        void updateVel(const double& dt, DynamicalState DS, ForcePtr force);
        //! partial solver: update position with collision
        void updatePosWithCollision(const double& dt, DynamicalState DS, GeometryPtr geom, const double& Cr, const double& Cs);

        std::string name;

    private:
        void updateSinglePos(const double& dt, DynamicalState DS, size_t i);
    };

    typedef SolverBase* SolverPtr;


//! ------------------------------------ LeapFrog -------------------------------------------------

    class LeapFrogSolver: public SolverBase
    {
    public:
        LeapFrogSolver()    {name="LeapFrog";}
        ~LeapFrogSolver()   {}

    protected:
        void _updateDS(const double& dt, DynamicalState DS, ForcePtr force);
        void _updateDSWithCollision(const double& dt, DynamicalState DS, ForcePtr force, GeometryPtr geom, const double& Cr, const double& Cs);
    };

//! ------------------------------------ SixthOrder -------------------------------------------------

    class SixOrderSolver: public LeapFrogSolver
    {
    public:
        SixOrderSolver()    {name="SixOrder";}
        ~SixOrderSolver()   {}

    protected:
        void _updateDS(const double& dt, DynamicalState DS, ForcePtr force);
        void _updateDSWithCollision(const double& dt, DynamicalState DS, ForcePtr force, GeometryPtr geom, const double& Cr, const double& Cs);
    };

}

#endif //PBA_FORCE_H
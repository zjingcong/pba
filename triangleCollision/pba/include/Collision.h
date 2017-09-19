//
// Created by jingcoz on 9/18/17.
//

#ifndef PBA_COLLISION_H
#define PBA_COLLISION_H

# include "DynamicalState.h"
# include "Geometry.h"

namespace pba
{
    class TriangleCollision
    {
    public:
        TriangleCollision(DynamicalState ds, Geom& geo, double delta):
                DS(ds),
                geom(geo),
                dt(delta),
                dt_i(0.0)   {}
        ~TriangleCollision()    {}

        void collision(const size_t p, const double& Cr, const double& Cs);

    private:
        DynamicalState DS;
        Geom geom;
        double dt;

        double dt_i;
        Vector x_i;
        TrianglePtr collision_triangle; // triangle where collision happens, match the maximum dt_i

        bool collisionDetection(const size_t p, TrianglePtr triangle);
        void collisionHandling(const size_t p, const double& Cr, const double& Cs);
    };

    typedef TriangleCollision* CollisionPtr;
}

#endif //PBA_COLLISION_H

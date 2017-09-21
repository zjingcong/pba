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
        static void triangleCollision(const double& dt, DynamicalState DS, const size_t i, GeometryPtr geom, const double& Cr, const double& Cs);

    private:
        //! collision detection per triangle
        static bool collisionDetection(const double& dt, DynamicalState DS, const size_t p, TrianglePtr triangle, double& dt_i, Vector& x_i);
        //! collision handling per triangle
        static void collisionHandling(const double& dt, DynamicalState DS, const size_t p, TrianglePtr collision_triangle, const double& dt_i, const Vector& x_i, const double& Cr, const double& Cs);
    };
}

#endif //PBA_COLLISION_H
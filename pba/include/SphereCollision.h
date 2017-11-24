//
// Created by jingcoz on 11/23/17.
//

#ifndef PBA_SPHERECOLLISION_H
#define PBA_SPHERECOLLISION_H

# include "Collision.h"
# include "SphereState.h"

namespace pba
{

    class SphereCollision: public CollisionBase
    {
    public:
        SphereCollision(SphereState ds): sphereDS(ds)   {}
        ~SphereCollision()  {}

        void collision(const double &dt);
        void collisionWithKdTree(const double &dt);

        void collisionWithinTriangles(const double& dt, const size_t i, std::vector<TrianglePtr> triangles, CollisionData& CD);

    private:
        SphereState sphereDS;

        //! collision detection per triangle
        void collisionDetection(const double& dt, const size_t p, TrianglePtr triangle, CollisionData& CD);
        //! collision handling per triangle
        void collisionHandling(const size_t p, const CollisionData& CD);

        bool isInTriangle(TrianglePtr triangle, const Vector& xi);
    };

    CollisionPtr CreateSphereCollision(SphereState ds);

}

#endif //PBA_SPHERECOLLISION_H

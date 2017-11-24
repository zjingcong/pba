//
// Created by jingcoz on 11/23/17.
//

#ifndef PBA_RBDCOLLISION_H
#define PBA_RBDCOLLISION_H

# include "Collision.h"
# include "Geometry.h"
# include "RigidBodyState.h"
# include "Matrix.h"

namespace pba
{

    class RBDCollision
    {
    public:
        static void RBD_Collision(const double& dt, const RigidBodyState& RBDS, pba::GeometryPtr geom);

    private:
        static bool collisionDetection(const double& dt, const RigidBodyState& RBDS, const size_t& p, TrianglePtr triangle, double& dt_i);
        static void collisionHandling(const RigidBodyState& RBDS, const CollisionData& CD);
        static void collision(const double &dt, const RigidBodyState &RBDS, std::vector<TrianglePtr> triangles, CollisionData &CD);
    };

}

#endif //PBA_RBDCOLLISION_H

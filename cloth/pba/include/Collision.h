//
// Created by jingcoz on 9/18/17.
//

# ifndef PBA_COLLISION_H
# define PBA_COLLISION_H

# include "DynamicalState.h"
# include "Geometry.h"
# include "RigidBodyState.h"
# include "Matrix.h"

namespace pba
{

    struct CollisionData
    {
        size_t id;  // collision particle
        double dt_i;
        Vector x_i;
        TrianglePtr triangle;
        bool collision_status;

        CollisionData():
                id(0),
                dt_i(0.0),
                x_i(Vector(0.0, 0.0, 0.0)),
                triangle(NULL),
                collision_status(false) {}
    };

    class TriangleCollision
    {
    public:
        //! collision detection within a list of triangles
        static void collisionWithinTriangles(const double& dt, DynamicalState DS, size_t i, std::vector<TrianglePtr> triangles, CollisionData& CD);
        //! collision detection and handling per geometry
        static void triangleCollision(const double& dt, DynamicalState DS, pba::GeometryPtr geom, const double& Cr, const double& Cs);
        //! collision detection and handling per KdTree stored geometry
        static void triangleCollisionWithKdTree(const double& dt, DynamicalState DS, pba::GeometryPtr geom, const double& Cr, const double& Cs);

    private:
        //! collision detection per triangle
        static bool collisionDetection(const double& dt, DynamicalState DS, const size_t p, TrianglePtr triangle, double& dt_i, Vector& x_i);
        //! collision handling per triangle
        static void collisionHandling(const double& dt, DynamicalState DS, const size_t p, const CollisionData& CD, const double& Cr, const double& Cs);
    };

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

# endif //PBA_COLLISION_H

//
// Created by jingcoz on 11/23/17.
//

#ifndef PBA_SPHERECOLLISION_H
#define PBA_SPHERECOLLISION_H

# include "Collision.h"
# include "SphereState.h"
# include <map>

namespace pba
{

    struct SphereCollisionData
    {
        size_t id1;  // collision sphere
        size_t id2; // sphere to collide with
        double dt_i;
        bool collision_status;
        bool detect_flag;

        SphereCollisionData(size_t id = 0):
                id1(id),
                id2(id),    // if id2 == id1: no assignment for id2
                dt_i(0.0),
                collision_status(false) {}
    };


    class SphereSphereCollision: public virtual CollisionBase
    {
    public:
        SphereSphereCollision(SphereState ds);
        ~SphereSphereCollision()    {}

        void init();
        void collision(const double &dt);

    private:
        SphereState sphereDS;
        std::vector<std::vector<bool>> detection_flags;

        void collisionDetection(const double& dt, const size_t i, const size_t j, SphereCollisionData& sphereCD);
        void collisionHandling(const size_t i, SphereCollisionData& sphereCD);

        bool timeTest(const double& dti, const double& dt);
        void reset_detection_flags(size_t i);
    };


    class SphereTriCollision: public virtual CollisionBase
    {
    public:
        SphereTriCollision(SphereState ds);
        ~SphereTriCollision()   {}

        void collision(const double &dt);
        void collisionWithKdTree(const double &dt);

        void collisionWithinTriangles(const double& dt, const size_t i, std::vector<TrianglePtr> triangles, CollisionData& CD);

    private:
        SphereState sphereDS;

        void collisionDetection(const double &dt, const size_t p, TrianglePtr triangle, CollisionData &CD);
        void collisionHandling(const size_t p, const CollisionData &CD);

        void triPlaneIntersectionTest(const double &dt, const size_t p, TrianglePtr triangle, CollisionData &CD);
        void edgeIntersectionTest(const double &dt, const size_t p, const Vector& P0, const Vector& P1, CollisionData &CD);
        void vertIntersectionTest(const double &dt, const size_t p, const Vector& P, CollisionData &CD);
        bool inTriangleTest(TrianglePtr triangle, const Vector &xi);
        bool timeTest(const double& dti, const double& dt);
    };


    class SphereCollision: public SphereSphereCollision, public SphereTriCollision
    {
    public:
        SphereCollision(SphereState ds);
        ~SphereCollision()  {}

        void init();
        void collision(const double &dt);
        void collisionWithKdTree(const double &dt);

        void collisionWithinTriangles(const double& dt, const size_t i, std::vector<TrianglePtr> triangles, CollisionData& CD);
    };

    CollisionPtr CreateSphereCollision(SphereState ds);

}

#endif //PBA_SPHERECOLLISION_H

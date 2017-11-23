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
                triangle(nullptr),
                collision_status(false) {}
    };


    class CollisionBase: public std::enable_shared_from_this<CollisionBase>
    {
    public:
        CollisionBase():
                // default elastic reflection
                Cr(1.0),
                Cs(1.0) {}
        ~CollisionBase()    {}

        const double& getCr() const { return Cr;}
        const double& getCs() const { return Cs;}
        void setCr(const double& cr)    {Cr = cr;}
        void setCs(const double& cs)    {Cs = cs;}
        void setGeom(GeometryPtr g)  {geom = g;}

        virtual void collisionWithinTriangles(const double& dt, const size_t i, std::vector<TrianglePtr> triangles, CollisionData& CD)    {};
        virtual void collision(const double& dt) {};
        virtual void collisionWithKdTree(const double& dt)   {};

    protected:
        double Cr;
        double Cs;
        GeometryPtr geom;

        virtual void collisionDetection(const double& dt, const size_t p, TrianglePtr triangle, CollisionData& CD)  {};
        virtual void collisionHandling(const double& dt, const size_t p, const CollisionData& CD)   {};
    };

    typedef std::shared_ptr<CollisionBase> CollisionPtr;


    class ParticleCollision: public CollisionBase
    {
    public:
        ParticleCollision(DynamicalState& ds): DS(ds)    {}
        ~ParticleCollision()    {}

        //! collision detection within a list of triangles
        void collisionWithinTriangles(const double& dt, const size_t i, std::vector<TrianglePtr> triangles, CollisionData& CD);
        //! collision detection and handling per geometry
        void collision(const double &dt);
        //! collision detection and handling per KdTree stored geometry
        void collisionWithKdTree(const double &dt);

    private:
        DynamicalState DS;

        //! collision detection per triangle
        void collisionDetection(const double& dt, const size_t p, TrianglePtr triangle, CollisionData& CD);
        //! collision handling per triangle
        void collisionHandling(const double& dt, const size_t p, const CollisionData& CD);
    };

    CollisionPtr CreateParticleCollision(DynamicalState& ds);


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

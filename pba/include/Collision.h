//
// Created by jingcoz on 9/18/17.
//

# ifndef PBA_COLLISION_H
# define PBA_COLLISION_H

# include "DynamicalState.h"
# include "Geometry.h"

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
        CollisionBase()
        {
            // default: elastic collision
            float_parms = {{"Cr", 1.0},
                           {"Cs", 1.0}};
        }
        ~CollisionBase()    {}

        const float get_floatParms(const std::string &key) const {return float_parms.at(key);}

        void set_parms(const std::string &key, const float &value)  {float_parms.at(key) = value;}
        void set_geom(GeometryPtr g)  {geom = g;}

        virtual void init()   {}
        virtual void collisionWithinTriangles(const double& dt, const size_t i, std::vector<TrianglePtr> triangles, CollisionData& CD)    {};
        virtual void collision(const double& dt) {};
        virtual void collisionWithKdTree(const double& dt)   {};

    protected:
        std::map<std::string, float> float_parms; // force floatParms
        GeometryPtr geom;

        virtual void collisionDetection(const double& dt, const size_t p, TrianglePtr triangle, CollisionData& CD)  {};
        virtual void collisionHandling(const size_t p, const CollisionData& CD)   {};
    };

    typedef std::shared_ptr<CollisionBase> CollisionPtr;


    class ParticleCollision: public CollisionBase
    {
    public:
        ParticleCollision(DynamicalState ds): DS(ds)    {}
        ~ParticleCollision()    {}

        //! collision detection and handling per geometry
        void collision(const double &dt);
        //! collision detection and handling per KdTree stored geometry
        void collisionWithKdTree(const double &dt);

        //! collision detection within a list of triangles
        void collisionWithinTriangles(const double& dt, const size_t i, std::vector<TrianglePtr> triangles, CollisionData& CD);

    private:
        DynamicalState DS;

        //! collision detection per triangle
        void collisionDetection(const double& dt, const size_t p, TrianglePtr triangle, CollisionData& CD);
        //! collision handling per triangle
        void collisionHandling(const size_t p, const CollisionData& CD);
    };

    CollisionPtr CreateParticleCollision(DynamicalState ds);

}

# endif //PBA_COLLISION_H

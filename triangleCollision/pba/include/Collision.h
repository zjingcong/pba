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
        TriangleCollision(GeometryPtr geo, const double& cr, const double& cs):
                geom(geo),
                Cr(cr),
                Cs(cs),
                dt_i(0.0) // init dt_i for comparing dt_i to get the maximum dt_i
        {}
        ~TriangleCollision()    {}

        void collision(const double& dt, DynamicalState DS, const size_t p);

        void updateCollisionCoefficient(const double& cr, const double& cs) {Cr = cr; Cs = cs;}
        double getCr() const { return Cr;}
        double getCs() const { return Cs;}
        const std::string& getGeomName() const { return geom->Name();}

    private:
        GeometryPtr geom;
        double Cr;
        double Cs;
        double dt_i;

        Vector x_i;
        TrianglePtr collision_triangle; // triangle where collision happens, match the maximum dt_i

        //! collision detection per triangle
        bool collisionDetection(const double& dt, DynamicalState DS, const size_t p, TrianglePtr triangle);
        void collisionHandling(const double& dt, DynamicalState DS, const size_t p);
    };

    typedef TriangleCollision* CollisionPtr;
}

#endif //PBA_COLLISION_H

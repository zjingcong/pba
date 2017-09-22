//
// Created by jingcoz on 9/21/17.
//

# ifndef PBA_BBOX_H
# define PBA_BBOX_H

# include "Vector.h"
# include "Geometry.h"
# include "assert.h"

namespace pba
{
    class AABB
    {
    public:
        AABB(Vector& llc, Vector& urc);
        ~AABB() {}

        void setLLC(const Vector& llc)  {LLC = llc;}
        void setURC(const Vector& urc)  {URC = urc;}

        const Vector& getLLC() const  { return LLC;}
        const Vector& getURC() const  { return URC;}

        //! AABB subdivision: 0 - X direction, 1 - Y direction, 2 - Z direction | id 0 - lowerleft, 1 - upperright
        AABB subDivide(const int i, const int id);
        bool insidePoint(const Vector& P);
        bool insideTriangle(TrianglePtr triangle);

        bool intersect() const;

    private:
        Vector LLC; // lower left center
        Vector URC; // upper right center

        bool lessVector(const Vector& v1, const Vector& v2);
    };
}

#endif //PBA_BBOX_H

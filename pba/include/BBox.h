//
// Created by jingcoz on 9/21/17.
//

# ifndef PBA_BBOX_H
# define PBA_BBOX_H

# include "Vector.h"
# include "Triangle.h"
# include "assert.h"

namespace pba
{
    class AABB
    {
    public:
        AABB();
        AABB(Vector& llc, Vector& urc);
        ~AABB() {}

        void setLLC(const Vector& llc)  {LLC = llc; center = (LLC + URC) * 0.5; vec_length = URC - LLC;}
        void setURC(const Vector& urc)  {URC = urc; center = (LLC + URC) * 0.5; vec_length = URC - LLC;}

        const Vector& getLLC() const  { return LLC;}
        const Vector& getURC() const  { return URC;}
        const Vector& getCenter() const { return center;}
        const Vector& getVecLength() const { return vec_length;}

        //! AABB subdivision: 0 - X direction, 1 - Y direction, 2 - Z direction | id 0 - lowerleft, 1 - upperright
        AABB subDivide(const int i, const int id);
        bool insidePoint(const Vector& P) const;
        //! return 0 - outside, 1 - inside, 2 - across
        int insideTriangle(TrianglePtr triangle) const;

        //! AABB intersection: 0 - no intersection, 1 - intersect, 2 - inside
        int intersect(const Vector& origin, const Vector& target) const;

    private:
        Vector LLC; // lower left center
        Vector URC; // upper right center
        Vector center;
        Vector vec_length;

        bool lessVector(const Vector& v1, const Vector& v2) const;
    };
}

#endif //PBA_BBOX_H

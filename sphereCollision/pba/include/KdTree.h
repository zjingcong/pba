//
// Created by jingcoz on 9/21/17.
//

# ifndef PBA_KDTREE_H
# define PBA_KDTREE_H

# include "BBox.h"
# include "Triangle.h"
# include "Vector.h"
# include "DynamicalState.h"

namespace pba
{
    struct CollisionData;
    class TriangleCollision;
}

# include <string>
# include <vector>

namespace pba
{

    class KdTree
    {
    public:
        KdTree(int d):
                level(-1),
                depth(d),
                left(NULL),
                right(NULL) {}
        ~KdTree()   {}

        int getDepth() const { return depth;}

        void build(AABB bbox, std::vector<pba::TrianglePtr> tris, int l = 0);
        pba::CollisionData searchCollision(double& dt, DynamicalState DS, size_t i, const Vector& origin, const Vector& target);
        std::vector<TrianglePtr> search(const Vector& origin, const Vector& target);

    private:
        AABB aabb;
        std::vector<pba::TrianglePtr> triangles;
        int level;
        int depth;
        KdTree* left;
        KdTree* right;

        std::vector<TrianglePtr> null_triangles;
    };

    typedef KdTree* KdTreePtr;

}

# include "Collision.h"
#endif //PBA_KDTREE_H

//
// Created by jingcoz on 9/21/17.
//

#ifndef PBA_KDTREE_H
#define PBA_KDTREE_H

# include "BBox.h"
# include "Triangle.h"
# include "Vector.h"

# include <string>
# include <vector>
# include <set>
# include <tuple>

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

        void build(AABB bbox, std::vector<pba::TrianglePtr> tris, int l = 0);
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

#endif //PBA_KDTREE_H

//
// Created by jingcoz on 9/21/17.
//

#ifndef PBA_KDTREE_H
#define PBA_KDTREE_H

# include "BBox.h"
# include "Geometry.h"
# include "Vector.h"

# include <string>

namespace pba
{

    class KdTree
    {
    public:
        KdTree(std::string& nam, int d, AABB& aabb):
                name{nam},
                root(NULL),
                depth(d),
                geomAABB(aabb)  {}
        ~KdTree()   {}

        const std::string& Name() const { return name; }
        void buildTree(std::vector<pba::TrianglePtr> tri);
        std::vector<pba::TrianglePtr> searchTriangles(const Vector& pos);

    private:
        std::string name;

        struct KdNode
        {
            std::vector<pba::TrianglePtr> triangles;
            pba::AABB bbox;
            KdNode *left;
            KdNode *right;

            KdNode(std::vector<pba::TrianglePtr> tri, pba::AABB aabb):
                    triangles(tri),
                    bbox(aabb),
                    left(NULL),
                    right(NULL) {}
        };

        KdNode *root;
        int depth;
        AABB geomAABB;

        void build(std::vector<pba::TrianglePtr> tri, pba::AABB aabb, KdNode*& t, int level);
        KdNode* search(const Vector& pos, pba::AABB aabb, KdNode*& t, int level);
    };

}

#endif //PBA_KDTREE_H

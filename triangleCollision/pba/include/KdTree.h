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

namespace pba
{

    class KdTree
    {
    public:
        KdTree(std::string& nam, int d):
                name{nam},
                root(NULL),
                depth(d)    {}
        ~KdTree()   {}

        const std::string& Name() const { return name; }
        int getDepth() const { return depth;}

        void buildTree(std::vector<TrianglePtr> tri, AABB aabb);
        std::vector<TrianglePtr> searchTriangles(const Vector& vec0, const Vector& vec1);

    private:
        std::string name;

        struct KdNode
        {
            std::vector<TrianglePtr> triangles;
            pba::AABB bbox;
            KdNode *left;
            KdNode *right;

            KdNode(std::vector<TrianglePtr> tri, pba::AABB aabb):
                    triangles(tri),
                    bbox(aabb),
                    left(NULL),
                    right(NULL) {}
        };

        KdNode *root;
        int depth;
        std::vector<TrianglePtr> null_triangles;

        void build(std::vector<TrianglePtr> tri, pba::AABB aabb, KdNode*& t, int level);
        std::vector<TrianglePtr> search(const Vector& vec0, const Vector& vec1, const float& t0, const float& t1, KdNode*& t, int level);
    };

    typedef KdTree* KdTreePtr;

}

#endif //PBA_KDTREE_H

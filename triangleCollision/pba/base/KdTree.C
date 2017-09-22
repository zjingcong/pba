//
// Created by jingcoz on 9/21/17.
//

# include "KdTree.h"

using namespace pba;
using namespace std;

void KdTree::buildTree(std::vector<pba::TrianglePtr> tri)
{
    build(tri, geomAABB, root, 0);
}

std::vector<pba::TrianglePtr> KdTree::searchTriangles(const Vector& pos)
{
    std::vector<pba::TrianglePtr> triangles;
    KdNode* n = search(pos, geomAABB, root, 0);
    if (n != NULL)
    {
        triangles = n->triangles;
    }

    return triangles;
}

void KdTree::build(std::vector<pba::TrianglePtr> tri, pba::AABB aabb, KdNode*& t, int level)
{
    if (level <= depth)
    {
        int direction = level % 3;
        AABB aabb0 = aabb.subDivide(direction, 0);
        AABB aabb1 = aabb.subDivide(direction, 1);
        std::vector<pba::TrianglePtr> triangles0;
        std::vector<pba::TrianglePtr> triangles1;
        for (auto it = tri.cbegin(); it != tri.cend(); ++it)
        {
            TrianglePtr triangle = *it;
            if (aabb0.insideTriangle(triangle)) {triangles0.push_back(triangle);}
            else if(aabb1.insideTriangle(triangle)) {triangles1.push_back(triangle);}
            else
            {
                triangles0.push_back(triangle);
                triangles1.push_back(triangle);
            }
        }
        if (!triangles0.empty())    {build(triangles0, aabb0, t->left, level + 1); tri.clear();}
        if (!triangles1.empty())    {build(triangles1, aabb1, t->right, level + 1); tri.clear();}
    }
}

KdTree::KdNode* KdTree::search(const Vector& pos, pba::AABB aabb, KdNode*& t, int level)
{
    if (t == NULL)  { return NULL;}
    if (level > depth)  { return NULL;}
    if (t->left == NULL && t->right == NULL)    { return t;}

    int direction = level % 3;
    AABB aabb0 = aabb.subDivide(direction, 0);
    AABB aabb1 = aabb.subDivide(direction, 1);

    if (aabb0.insidePoint(pos)) { return search(pos, aabb0, t->left, level + 1);}
    else if (aabb1.insidePoint(pos))    { return search(pos, aabb1, t->right, level + 1);}

    return NULL;
}

//
// Created by jingcoz on 9/21/17.
//

# include "KdTree.h"
# include <iostream>

using namespace pba;
using namespace std;

void KdTree::buildTree(std::vector<TrianglePtr> tri, AABB aabb)
{
    build(tri, aabb, root, 0);
    cout << "KdTree building complete | Tree Depth: " << depth << endl;
}

std::vector<TrianglePtr> KdTree::searchTriangles(const Vector& vec0, const Vector& vec1)
{
    float t0 = 0.0;
    float t1 = float((vec1 - vec0).magnitude());
    Vector dir = vec1 - vec0;
    return search(vec0, dir, t0, t1, root, 0);
}

void KdTree::build(std::vector<TrianglePtr> tri, pba::AABB aabb, KdNode*& t, int level)
{
    if (t == NULL)  {t = new KdNode(tri, aabb);}
    else if (level == depth) { t = new KdNode(tri, aabb); }
    else if (level < depth)
    {
        int direction = level % 3;
        AABB aabb0 = aabb.subDivide(direction, 0);
        AABB aabb1 = aabb.subDivide(direction, 1);
        std::vector<TrianglePtr> triangles0;
        std::vector<TrianglePtr> triangles1;
        for (auto it = tri.cbegin(); it != tri.cend(); ++it)
        {
            TrianglePtr triangle = *it;
            if (aabb0.insideTriangle(triangle) != 0) {triangles0.push_back(triangle);}
            if (aabb1.insideTriangle(triangle) != 0) {triangles1.push_back(triangle);}
        }
        if (!triangles0.empty())    {build(triangles0, aabb0, t->left, level + 1); tri.clear();}
        if (!triangles1.empty())    {build(triangles1, aabb1, t->right, level + 1); tri.clear();}
    }
}

std::vector<TrianglePtr> KdTree::search(const Vector& origin, const Vector& dir, const float& t0, const float& t1, KdNode*& t, int level)
{
    if (t == NULL)  { return null_triangles;}
    if (level > depth)  { return null_triangles;}
    if ((t->left == NULL) && (t->right == NULL))    { return t->triangles;}

    if ((t == root) && (!(t->bbox.intersect(origin, dir, t0, t1))))  { return null_triangles;}
    else
    {
        bool left_result = t->left->bbox.intersect(origin, dir, t0, t1);
        bool right_result = t->right->bbox.intersect(origin, dir, t0, t1);
        if (left_result && right_result)
        {
            std::vector<TrianglePtr> triangles0 = search(origin, dir, t0, t1, t->left, level + 1);
            std::vector<TrianglePtr> triangles1 = search(origin, dir, t0, t1, t->right, level + 1);
            triangles0.insert(triangles0.end(), triangles1.begin(), triangles1.end());  // combine two triangles containers
            return triangles0;
        }

        if (left_result)
        { return search(origin, dir, t0, t1, t->left, level + 1); }
        if (right_result)
        { return search(origin, dir, t0, t1, t->right, level + 1); }
    }

    return null_triangles;
}

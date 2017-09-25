//
// Created by jingcoz on 9/21/17.
//

# include "KdTree.h"
# include <iostream>

using namespace pba;
using namespace std;

void KdTree::build(AABB bbox, std::vector<pba::TrianglePtr> tris, int l)
{
    aabb = bbox;
    level = l;
    left = NULL;
    right = NULL;
    cout << "Current level: " << level << endl;

    // end conditions
    if(tris.empty()) { cout << "Current triangles num: " << tris.size() << endl; return; }
    if (l == depth) { triangles = tris; cout << "Current triangles num: " << tris.size() << endl; return; }

    // compute
    int direction = level % 3;
    AABB aabb_left = aabb.subDivide(direction, 0);
    AABB aabb_right = aabb.subDivide(direction, 1);

    std::vector<TrianglePtr> triangles_left;
    std::vector<TrianglePtr> triangles_right;
    // split triangles
    for (auto it = tris.cbegin(); it != tris.cend(); ++it)
    {
        TrianglePtr triangle = *it;
        if (aabb_left.insideTriangle(triangle) == 1) {triangles_left.push_back(triangle);}
        else if (aabb_right.insideTriangle(triangle) == 1) {triangles_right.push_back(triangle);}
        else
        {
            triangles_left.push_back(triangle);
            triangles_right.push_back(triangle);
        }
    }

    left = new KdTree(depth);
    right = new KdTree(depth);
    left->build(aabb_left, triangles_left, level + 1);
    right->build(aabb_right, triangles_right, level + 1);
}

std::vector<TrianglePtr> KdTree::search(const Vector& origin, const Vector& target)
{
    int intersect_result = aabb.intersect(origin, target);

    if (intersect_result == 0) { return null_triangles; }
    if (left == NULL && right == NULL) { return triangles; }
    if (level == depth) { return triangles; }

    std::vector<TrianglePtr> triangles_left = left->search(origin, target);
    std::vector<TrianglePtr> triangles_right = right->search(origin, target);

    // combine two triangles containers
    triangles_left.insert(triangles_left.end(), triangles_right.begin(), triangles_right.end());

    return triangles_left;
}

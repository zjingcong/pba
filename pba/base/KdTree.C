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

    // end conditions
    if(tris.empty()) { return; }
    if (l == depth) { triangles = tris; return; }

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

pba::CollisionData KdTree::searchCollision(double& dt, DynamicalState DS, size_t i, const Vector& origin, const Vector& target)
{
    int intersect_result = aabb.intersect(origin, target);
    CollisionData collisionData;
    if (intersect_result == 0) { collisionData.collision_status = false;    return collisionData;}
    if ((left == NULL && right == NULL) || (level == depth))
    {
        TriangleCollision::collisionWithinTriangles(dt, DS, i, triangles, collisionData);
        return collisionData;
    }

    CollisionData collisionData_left = left->searchCollision(dt, DS, i, origin, target);
    CollisionData collisionData_right = right->searchCollision(dt, DS, i, origin, target);
    collisionData.collision_status = collisionData_left.collision_status + collisionData_right.collision_status;
    if (std::fabs(collisionData_left.dt_i) > std::fabs(collisionData_right.dt_i))
    {
        collisionData.dt_i = collisionData_left.dt_i;
        collisionData.triangle = collisionData_left.triangle;
        collisionData.x_i = collisionData_left.x_i;
    }
    else
    {
        collisionData.dt_i = collisionData_right.dt_i;
        collisionData.triangle = collisionData_right.triangle;
        collisionData.x_i = collisionData_right.x_i;
    }

    return collisionData;
};

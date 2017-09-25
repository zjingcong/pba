//
// Created by jingcoz on 9/18/17.
//

#ifndef PBA_GEOMETRY_H
#define PBA_GEOMETRY_H

# include <vector>
# include <string>
# include <memory>
# include "Vector.h"
# include "Color.h"
# include "Triangle.h"

namespace pba
{
    class KdTree;
    typedef KdTree* KdTreePtr;
}

# include "BBox.h"

namespace pba
{

    class TriangleGeometry
    {
    public:
        TriangleGeometry(const std::string& nam): name(nam) {}
        ~TriangleGeometry() {}

        void add_triangle(TrianglePtr tri);
        void setBBox(AABB aabb) {bbox.setLLC(aabb.getLLC()); bbox.setURC(aabb.getURC());}
        void setBBox(Vector& llc, Vector& urc)  {bbox.setLLC(llc); bbox.setURC(urc);}

        std::vector<TrianglePtr>& get_triangles()    { return triangles;}   // return ref!
        AABB getBBox() const { return bbox;}
        KdTreePtr getKdTree() const  { return trianglesTree;}
        size_t get_nb() const { return triangles.size();}
        const std::string& Name() const { return name;}

        void cleanTrianglesCollisionStatus();

        void build_triangles(std::vector<Vector> &vertices, std::vector<Vector> &face_indices);  // here face_indices start with 0
        void build_trianglesTree(int depth);

    private:
        std::string name;
        std::vector<TrianglePtr> triangles;
        KdTreePtr trianglesTree;
        AABB bbox;

        TriangleGeometry()  {}
    };

    typedef TriangleGeometry* GeometryPtr;
};

# include "KdTree.h"
#endif //PBA_GEOMETRY_H

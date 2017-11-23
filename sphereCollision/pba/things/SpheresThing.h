//
// Created by jingcoz on 11/22/17.
//

#ifndef PBA_SPHERESTHING_H
#define PBA_SPHERESTHING_H

# include "PbaThing.h"
# include "Tools.h"
# include "SphereState.h"

# ifdef __APPLE__
# include <OpenGL/gl.h>   // OpenGL itself.
  #include <OpenGL/glu.h>  // GLU support library.
  #include <GLUT/glut.h>
# else
# include <GL/gl.h>   // OpenGL itself.
# include <GL/glu.h>  // GLU support library.
# include <GL/glut.h> // GLUT support library.

# endif

namespace pba
{

    class SpheresThing: public PbaThingyDingy
    {
    public:
        SpheresThing(const std::string nam = "SpheresInWellThing") :
                PbaThingyDingy(nam)
        {
            geom = CreateGeometry("wellGeom");
            sphereDS = CreateSphereState("sphereDS");
        }

        ~SpheresThing() {}

        void Init(const std::vector<std::string> &args)
        {
            /// load geometry
            LoadMesh::LoadObj("../../models/well.obj", geom);
            // set geometry color
            for (auto &it: geom->get_triangles()) {
                it->setColor(Color(float(drand48() * 0.4 + 0.6), 0.5, float(drand48() * 0.4 + 0.6), 1.0));
            }
            geom->build_trianglesTree(6);   // build geom kdTree
            std::cout << "-------------------------------------------" << std::endl;

            /// load spheres

        }

        void Reset() {}

        void solve() {}

        void Display()
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            Draw::DrawTriangles(geom, false);
            glFlush();
        }

        void Keyboard(unsigned char key, int x, int y) {}

        void Usage() {}

    private:
        // well geometry
        GeometryPtr geom;
        // sphere dynamical state
        SphereState sphereDS;
    };

    pba::PbaThing Spheres() { return PbaThing(new pba::SpheresThing()); }
}

#endif //PBA_SPHERESTHING_H

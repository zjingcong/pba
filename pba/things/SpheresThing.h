//
// Created by jingcoz on 11/22/17.
//

#ifndef PBA_SPHERESTHING_H
#define PBA_SPHERESTHING_H

# include "PbaThing.h"
# include "Tools.h"
# include "SphereState.h"
# include "Solver.h"
# include "SphereCollision.h"

# ifdef __APPLE__
# include <OpenGL/gl.h>   // OpenGL itself.
  #include <OpenGL/glu.h>  // GLU support library.
  #include <GLUT/glut.h>
# else
# include <GL/gl.h>   // OpenGL itself.
# include <GL/glu.h>  // GLU support library.
# include <GL/glut.h> // GLUT support library.

# endif

# define LEAP_FROG 0
# define SIX_ORDER 1

namespace pba
{

    class SpheresThing: public PbaThingyDingy
    {
    public:
        SpheresThing(const std::string nam = "SpheresInWellThing") :
                PbaThingyDingy(nam),
                kdtree_level(6),
                solver_id(LEAP_FROG),
                g(0.98)
        {
            geom = CreateGeometry("wellGeom");
            sphereDS = CreateSphereState("sphereDS");
            collision = CreateSphereCollision(sphereDS);
            solver_list.push_back(CreateLeapFrogSolver());
            solver_list.push_back(CreateSixOrderSolver());
            solver_list[LEAP_FROG]->setCollision(collision);
            solver_list[SIX_ORDER]->setCollision(collision);
        }

        ~SpheresThing() {}

        void Init(const std::vector<std::string> &args)
        {
            /// load geometry
            LoadMesh::LoadObj("../../models/well.obj", geom);
            // set geometry color
            for (auto &it: geom->get_triangles()) {
                it->setColor(Color(float(drand48() * 0.4 + 0.6), float(drand48() * 0.4 + 0.6), float(drand48() * 0.4 + 0.6), 1.0));
            }
            geom->build_trianglesTree(kdtree_level);   // build geom kdTree
            std::cout << "-------------------------------------------" << std::endl;

            /// load spheres
            add_spheres(2);
            std::cout << "-------------------------------------------" << std::endl;

            /// load forces
            gravity = CreateGravity(sphereDS, g); // add gravity
            std::cout << "- Add force " << gravity->Name() << std::endl;
            forces.push_back(gravity);
            std::cout << "-------------------------------------------" << std::endl;
        }

        void Reset() {}

        void solve() {}

        void Display()
        {
//            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//            Draw::DrawTriangles(geom, false);
            draw_spheres();
//            glFlush();
        }

        void Keyboard(unsigned char key, int x, int y) {}

        void Usage() {}

    private:
        // well geometry
        GeometryPtr geom;
        int kdtree_level;
        // sphere dynamical state
        SphereState sphereDS;
        // collision
        CollisionPtr collision;
        // solvers
        SolverPtr solver;
        std::vector<SolverPtr> solver_list;
        // force
        ForcePtr gravity;
        ForcePtrContainer forces;

        // keyboard controls
        int solver_id;
        float g;

        void add_spheres(size_t sphere_num)
        {
            size_t nb = sphereDS->nb();
            sphereDS->add(sphere_num);
            for (size_t i = 0; i < sphere_num; ++i)
            {
                size_t id = nb + i;
                sphereDS->set_id(id, int(id));
                sphereDS->set_radius(id, 0.1);
                sphereDS->set_pos(id, 2 * Vector((drand48() - 0.5), (drand48() - 0.5), (drand48() - 0.5)));  // default position
                sphereDS->set_vel(id, Vector((drand48() - 0.5)* 0.5, 0.0, (drand48() - 0.5) * 0.5));  // default velocity
                sphereDS->set_ci(id, Color(0.0, float(drand48() * 0.75 + 0.25), float(drand48() * 0.75 + 0.25), 1.0));    // random color
            }
        }

        void draw_spheres()
        {
            glLoadIdentity();
            for (size_t i = 0; i < sphereDS->nb(); ++i)
            {
                float radius = sphereDS->radius(i);
                Vector pos = sphereDS->pos(i);
                Color color = sphereDS->ci(i);
                glTranslatef(pos.X(), pos.Y(), pos.Z());
                glColor3f(color.X(), color.Y(), color.Z());
                glutSolidSphere(radius, 20, 20);
            }
        }
    };

    pba::PbaThing Spheres() { return PbaThing(new pba::SpheresThing()); }
}

#endif //PBA_SPHERESTHING_H

//
// Created by jingcoz on 10/20/17.
//

// This program assemble a rigid body dynamics system in which the rigid body is initialized to points on the surface of specified geometry
// - be able to read an obj file and initialize the particle positions at the vertices of the obj file.
// - contain the RBD object within a cube that the RBD object collides with.
// - The RBD object should start near (0,0,0) with an initial center of mass velocity and angular velocity.
// - Keys should control the magnitude of the initial velocities.

#ifndef PBA_RBDTHING_H
#define PBA_RBDTHING_H

# include "PbaThing.h"
# include "RigidBodyState.h"
# include "Tools.h"
# include "Force.h"
# include "RBD.h"

# ifdef __APPLE__
# include <OpenGL/gl.h>   // OpenGL itself.
  #include <OpenGL/glu.h>  // GLU support library.
  #include <GLUT/glut.h>
# else
# include <GL/gl.h>   // OpenGL itself.
# include <GL/glu.h>  // GLU support library.
# include <GL/glut.h> // GLUT support library.
# endif

namespace pba {

    class RigidBodyDynamicThing: public PbaThingyDingy
    {
    public:
        RigidBodyDynamicThing(const std::string nam = "RBDThing"):
                PbaThingyDingy(nam)
        {
            RBDS = CreateRigidBodyState("RBDS");
            // force
            gravity = CreateGravity(0.98);
            forces.push_back(gravity);
            // solver
            solver = CreateRBDLeapFrogSolver();
        }
        ~RigidBodyDynamicThing()    {}

        void Init(const std::vector<std::string>& args)
        {
            /// load scene
            // load geometry
            geom = CreateGeometry("collisionMesh");
            LoadMesh::LoadBox(6, geom);
            geom->build_trianglesTree(0);   // build geom kdTree
            // set geometry color
            for (auto it = geom->get_triangles().begin(); it != geom->get_triangles().end(); ++it)
            {
                TrianglePtr triangle = *it;
                triangle->setColor(
                        Color(float(drand48()), float(drand48()), float(drand48()), 1.0));   // set random colors
            }
            cout << "-------------------------------------------" << endl;

            /// load .obj file for rigid body
            std::vector<Vector> verts;
            if (args.size() == 2)
            {
                std::string scene_file = args[1];
                LoadMesh::LoadObj(scene_file, verts);
            }
            else { std::cout << "Please specify rigid body model." << std::endl; exit(0);}

            // specify mass
            std::vector<double> m;
            for (size_t i = 0; i < verts.size(); ++i)   {m.push_back(1.0);}
            std::cout << "Set default mass for each particles: " << 1.0 << std::endl;

            // init rigid body state data
            Vector v_cm = Vector(1.0, -1.0, 0.0);
            Vector v_ang = Vector(0.0, 0.0, 0.0);
            RBDS->set_init(verts, m, v_cm, v_ang);
            std::cout << "Init vel_cm: "; v_cm.printValue(); std::cout << std::endl;
            std::cout << "Init v_ang: "; v_ang.printValue(); std::cout << std::endl;

            // set random colors
            for (size_t i = 0; i < RBDS->nb(); ++i)
            {RBDS->set_ci(i, Color(0.0, float(drand48() * 0.75 + 0.25), float(drand48() * 0.75 + 0.25), 1.0));}
        }

        void Reset()
        {

        }

        void solve()
        {
            solver->updateRBDS(dt, RBDS, forces); // no collision
        }

        void Display()
        {
            /// draw scene
            Draw::DrawTriangles(geom);

            /// draw rigid body particles
            glPointSize(1.0f);
            glBegin(GL_POINTS);
            for (size_t i = 0; i < RBDS->nb(); ++i)
            {
                // Vector pos = RBDS->pos(i);
                Vector pos = RBDS->vert_pos(i);
                Color color = RBDS->ci(i);
                glColor3f(color.red(), color.green(), color.blue());
                glVertex3f(float(pos.X()), float(pos.Y()), float(pos.Z()));
            }
            glEnd();
            glFlush();
        }

        void Keyboard(unsigned char key, int x, int y)
        {

        }

        void Usage()
        {

        }


    private:
        RigidBodyState RBDS;
        ForcePtrContainer forces;
        ForcePtr gravity;
        RBDSolverPtr solver;
        GeometryPtr geom;
    };

    //! create pbathing
    pba::PbaThing RBD() { return PbaThing(new pba::RigidBodyDynamicThing()); }

}

#endif //PBA_RBD_H

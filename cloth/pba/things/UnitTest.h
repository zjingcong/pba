//
// Created by jingcoz on 10/20/17.
//

// This program assembles a rigid body dynamics system in which the rigid body is initialized to points on the surface of specified geometry
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

    class UnitTestThing: public PbaThingyDingy
    {
    public:
        UnitTestThing(const std::string nam = "RBDThing"):
                PbaThingyDingy(nam),
                vel_cm_mag(1.0),
                vel_ang_mag(0.1),
                g(0.98),
                substep(1)
        {
            dt = 1.0/24;
            // rigid body data
            RBDS = CreateRigidBodyState("RBDS");
            // force
            gravity = CreateGravity(g);
            forces.push_back(gravity);
            // solver
            solver = CreateRBDLeapFrogSolver();
            subSolver = CreateRBDSubSolver();
            subSolver->setSolver(solver);
        }
        ~UnitTestThing()    {}

        void Init(const std::vector<std::string>& args)
        {
            float box_len = 6;
            /// load scene
            // load geometry
//            geom = CreateGeometry("collisionCube");
//            LoadMesh::LoadBox(box_len, geom);
//            // set geometry color
//            for (auto it = geom->get_triangles().begin(); it != geom->get_triangles().end(); ++it)
//            {
//                TrianglePtr triangle = *it;
//                triangle->setColor(Color(float(drand48() * 0.4 + 0.6), 0.5, float(drand48() * 0.4 + 0.6), 1.0));   // set random colors
//            }
            geom = CreateGeometry("collisionPlane");
            LoadMesh::LoadPlane(Vector(3.0, -6.0, -3.0), 20, 13, geom);
            for (auto it = geom->get_triangles().begin(); it != geom->get_triangles().end(); ++it)
            {
                TrianglePtr triangle = *it;
                triangle->setColor(Color(float(drand48() * 0.4 + 0.6), 0.5, float(drand48() * 0.4 + 0.6), 1.0));   // set random colors
            }
            std::cout << "-------------------------------------------" << std::endl;

            /// load .obj file for rigid body
            AABB bbox;
            if (args.size() == 2)
            {
                std::string scene_file = args[1];
                LoadMesh::LoadObj(scene_file, verts, bbox);
            }
            else { std::cout << "Please specify rigid body model." << std::endl; exit(0);}
            std::cout << "-------------------------------------------" << std::endl;
            
            // scale rigid body to fit box
            double len = bbox.getVecLength().magnitude();
            double scale = 13.0 / (box_len * len);
            std::cout << "Scale rigid body model: " << scale << std::endl;
            for (size_t i = 0; i < verts.size(); ++i)   {verts[i] = verts[i] * scale;}

            /// init rigid body data
            // specify mass
            for (size_t i = 0; i < verts.size(); ++i)   {m.push_back(1.0);}
            std::cout << "Set default mass for each particles: " << 1.0 << std::endl;

            // init rigid body state data
            Vector v_cm = Vector(0.0, -1.0, 0.0).unitvector() * vel_cm_mag;
            Vector v_ang = Vector(1.0, 1.0, 1.0).unitvector() * vel_ang_mag;
            RBDS->Init(verts, m, v_cm, v_ang);

            // set random colors
            for (size_t i = 0; i < RBDS->nb(); ++i)
            {RBDS->set_ci(i, Color(0.0, float(drand48() * 0.75 + 0.25), float(drand48() * 0.75 + 0.25), 1.0));}

            std::cout << "-------------------------------------------" << std::endl;
        }

        void Reset()
        {
            Vector v_cm = Vector(1.0, -1.0, 1.0) * vel_cm_mag;
            Vector v_ang = Vector(0.1, 0.1, 0.1) * vel_ang_mag;
            RBDS->Reset(v_cm, v_ang);
            printAttri();
            std::cout << "-------------------------------------------" << std::endl;
        }

        void solve()
        {
            // clean collision status
            geom->cleanTrianglesCollisionStatus();
            // leap frog solver
            // solver->updateRBDS(dt, RBDS, forces); // no collision
            // solver->updateRBDSWithCollision(dt, RBDS, forces, geom); // collision
            // subsolver
            subSolver->updateRBDSWithCollision(dt, RBDS, forces, geom); // subsolver
        }

        void Display()
        {
            /// draw scene
            Draw::DrawTriangles(geom);

            /// draw rigid body particles
            glPointSize(2.8f);
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
            if (key >= '1' && key <= '9')
            {
                substep = int(key) - 48;
                std::cout << "Set sub step: " << substep << std::endl;
                subSolver->setSubstep(substep);
                return;
            }

            switch (key)
            {
                /// velocities magnitude control
                case 'v': { vel_cm_mag /= 1.1; std::cout << "init center of mass velocity magnitude: " << vel_cm_mag << std::endl; break; }
                case 'V': { vel_cm_mag *= 1.1; std::cout << "init center of mass velocity magnitude: " << vel_cm_mag << std::endl; break; }

                case 'w': { vel_ang_mag /= 1.1; std::cout << "init angular velocity magnitude: " << vel_ang_mag << std::endl; break; }
                case 'W': { vel_ang_mag *= 1.1; std::cout << "init angular velocity magnitude: " << vel_ang_mag << std::endl; break; }

                /// gravity control
                case 'g':
                { g /= 1.05; gravity->update_parms("g", g);  std::cout << "gravity constant: " << g << std::endl; break; }
                case 'G':
                { g *= 1.05; gravity->update_parms("g", g);  std::cout << "gravity constant: " << g << std::endl; break; }

                /// timestep control
                case 't':
                { dt /= 1.1; std::cout << "time step " << dt << std::endl; break;}
                case 'T':
                { dt *= 1.1; std::cout << "time step " << dt << std::endl; break;}

                /// mesh display mode
                case 'l':
                {
                    wireframe = !wireframe;
                    if (!wireframe)  { glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); std::cout << "Display mode: normal" << std::endl;}
                    else    { glPolygonMode( GL_FRONT_AND_BACK, GL_LINE); std::cout << "Display mode: wireframe" << std::endl;}
                    break;
                }

                /// quit
                case 27: { exit(0); }

                default:
                    break;
            }
        }

        void Usage()
        {
            std::cout << "=== PbaThing ===" << std::endl;
            std::cout << "v/V     reduce/increase center of mass velocity magnitude" << std::endl;
            std::cout << "w/W     reduce/increase angular velocity magnitude" << std::endl;
            std::cout << "t/T     reduce/increase time step" << std::endl;
            std::cout << "g/G     reduce/increase gravity constant" << std::endl;
            std::cout << "l       switch between wireframe and normal display mode" << std::endl;
            std::cout << "1-9     specify sub step" << std::endl;
            std::cout << "Esc     quit" << std::endl;
        }


    private:
        RigidBodyState RBDS;
        ForcePtrContainer forces;
        ForcePtr gravity;
        RBDSolverPtr solver;
        GeometryPtr geom;
        RBDSubSolverPtr subSolver;

        //! rigid body init data
        std::vector<Vector> verts;
        std::vector<double> m;

        //! keyboard control
        float vel_cm_mag;   // magnitude of center of mass velocity
        float vel_ang_mag;  // magnitude of angular velocity
        float g;    // gravity constant
        bool wireframe; // mesh display mode
        int substep;

        void printAttri()
        {
            std::cout << "vel_cm_mag:  " << vel_cm_mag << std::endl;
            std::cout << "vel_ang_mag: " << vel_ang_mag << std::endl;
            std::cout << "time step:   " << dt << std::endl;
            std::cout << "g:           " << g << std::endl;
            std::cout << "substep:     " << substep << std::endl;
        }
    };

    //! create pbathing
    pba::PbaThing UnitTest() { return PbaThing(new pba::UnitTestThing()); }

}

#endif //PBA_RBD_H

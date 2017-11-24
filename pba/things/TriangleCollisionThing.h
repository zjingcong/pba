//
// Created by jingcoz on 9/13/17.
//

// Usage:
//  - collision coefficient of restitution (keys c/C to reduce/increase)
//  - collision coefficient of stickiness (keys s/S to reduce/increase)
//  - number of particles (key e to start/stop emitting more particles)
//  - magnitude of gravity (keys g/G to reduce/increase the magnitude)
//  - timestep size (keys t/T to reduce/increase) - this is already implemented in PbaThingyDingy
//  - mesh display mode (key w to switch wireframe and normal mode)
//  - KdTree (key k to turn on/off kdtree)
//  - Esc to quit

# include "PbaThing.h"
# include "Vector.h"
# include "Color.h"
# include "Solver.h"
# include "Force.h"
# include "DynamicalState.h"
# include "Tools.h"
# include "Geometry.h"
# include "Collision.h"

# ifdef __APPLE__
#include <OpenGL/gl.h>   // OpenGL itself.
  #include <OpenGL/glu.h>  // GLU support library.
  #include <GLUT/glut.h>
# else
# include <GL/gl.h>   // OpenGL itself.
# include <GL/glu.h>  // GLU support library.
# include <GL/glut.h> // GLUT support library.
# endif

# include <iostream>

# define LEAP_FROG 0
# define SIX_ORDER 1


namespace pba {

    class TriangleCollisionThing: public PbaThingyDingy
    {
    public:
        TriangleCollisionThing(const std::string nam = "TriangleCollisionThing"):
                PbaThingyDingy(nam),
                solver_id(LEAP_FROG),
                addParticles(false),
                wireframe(false),
                onKdTree(false)
        {
            // construct attributes
            _init();
            // construct dynamical state
            DS = CreateDynamicalState("collisionParticles");
            // construct solvers
            solver_list.push_back(CreateLeapFrogSolver());
            solver_list.push_back(CreateSixOrderSolver());
            solver = solver_list[LEAP_FROG];
            collision = CreateParticleCollision(DS);
            solver_list[0]->setCollision(collision);
            solver_list[0]->setCollision(collision);
            // add force
            addForce();
        }

        ~TriangleCollisionThing() {}

        void Init(const std::vector<std::string>& args)
        {
            /// load scene
            // load geometry
            geom = CreateGeometry("CollisionGeo");
            if (args.size() == 2)   // load .obj file
            {
                std::string scene_file = args[1];
                LoadMesh::LoadObj(scene_file, geom);
                geom->build_trianglesTree(6);   // build geom kdTree
            }
            // load cube
            else
            {
                LoadMesh::LoadBox(10, geom);
                geom->build_trianglesTree(0);   // build geom kdTree
            }
            // set geometry color
            for (auto it = geom->get_triangles().begin(); it != geom->get_triangles().end(); ++it)
            {
                TrianglePtr triangle = *it;
                triangle->setColor(Color(float(drand48() * 0.4 + 0.6), 0.5, float(drand48() * 0.4 + 0.6), 1.0));   // set random colors
            }
            std::cout << "-------------------------------------------" << std::endl;

            /// init sim
            // init dynamical state
            emitParticles(num);
            collision->setGeom(geom);
            collision->setCr(Cr);
            collision->setCs(Cs);
        }

        void Reset()
        {
            // set default values
            _init();
            DS->clear();
            // init sim
            emitParticles(num);
        }

        void solve()
        {
            // clean collision status
            geom->cleanTrianglesCollisionStatus();
            // emit particles
            if (addParticles)
            {
                size_t increase_num = 10;
                emitParticles(increase_num);
                std::cout << "particles number: " << DS->nb() << std::endl;
            }

            // update dynamical state
            if (onKdTree)   {solver->updateDSWithCollisionWithKdTree(dt, DS, forces);}
            else    {solver->updateDSWithCollision(dt, DS, forces);} // collision
        }

        void Display()
        {
            // draw scene
            Draw::DrawTriangles(geom);
            // draw particles
            glPointSize(4.2f);
            glBegin(GL_POINTS);
            for (size_t i = 0; i < DS->nb(); ++i)
            {
                Vector pos = DS->pos(i);
                Color color = DS->ci(i);
                glColor3f(color.red(), color.green(), color.blue());
                glVertex3f(float(pos.X()), float(pos.Y()), float(pos.Z()));
            }
            glEnd();
            glFlush();
        }

        void Keyboard(unsigned char key, int x, int y)
        {
            switch (key)
            {
                /// gravity control
                case 'g':
                { g /= 1.1; std::cout << "gravity constant: " << g << std::endl; gravity->update_parms("g", g); break; }
                case 'G':
                { g *= 1.1; std::cout << "gravity constant: " << g << std::endl; gravity->update_parms("g", g); break; }

                /// timestep control
                case 't':
                { dt /= 1.1; std::cout << "time step " << dt << std::endl; break;}
                case 'T':
                { dt *= 1.1; std::cout << "time step " << dt << std::endl; break;}

                /// solver switch
                case 'q':
                {
                    solver_id += 1;
                    solver_id = (solver_id % 2);
                    solver = solver_list[solver_id];
                    std::cout << "Current Solver: " << solver->Name() << std::endl;
                    break;
                }

                /// particles number control
                case 'e':
                {
                    addParticles = !addParticles;
                    if (addParticles) { std::cout << "START EMITTING" << std::endl; }
                    else { std::cout << "STOP EMITTING" << std::endl; }
                    break;
                }

                /// collision coefficients control
                case 'c':   // Cr
                { Cr /= 1.1;    collision->setCr(Cr);   std::cout << "coefficient of restitution Cr: " << Cr << std::endl;  break; }
                case 'C':
                {
                    Cr *= 1.1;
                    if (Cr >= 1.0)  {Cr = 1.0;}
                    collision->setCr(Cr);
                    std::cout << "coefficient of restitution Cr: " << Cr << std::endl;
                    break;
                }
                case 's':   // Cs
                { Cs /= 1.1;    collision->setCs(Cs);   std::cout << "coefficient of stickiness  Cs: " << Cs << std::endl;  break; }
                case 'S':
                {
                    Cs *= 1.1;
                    if (Cs >= 1.0)  {Cs = 1.0;}
                    collision->setCs(Cs);
                    std::cout << "coefficient of stickiness  Cs: " << Cs << std::endl;
                    break;
                }

                /// mesh display mode
                case 'w':
                {
                    wireframe = !wireframe;
                    if (!wireframe)  { glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); }
                    else    { glPolygonMode( GL_FRONT_AND_BACK, GL_LINE); }
                    break;
                }

                /// kdtree
                case 'k':
                {
                    onKdTree = !onKdTree;
                    if (onKdTree)   {std::cout << "TURN ON KDTREE" << std::endl;}
                    else    {std::cout << "TURN OFF KDTREE" << std::endl;}
                    break;
                }

                /// quit
                case 27:
                { exit(0);}

                default:
                    break;
            }
        }

        void Usage()
        {
            std::cout << "=== PbaThing ===" << std::endl;
            std::cout << "c/C     reduce/increase collision coefficient of restitution" << std::endl;
            std::cout << "s/S     reduce/increase collision coefficient of stickiness" << std::endl;
            std::cout << "g/G     reduce/increase magnitude of gravity" << std::endl;
            std::cout << "e       start/stop emitting more particles" << std::endl;
            std::cout << "t/T     reduce/increase animation time step" << std::endl;
            std::cout << "q       switch solvers between Leap Frog and Sixth Order" << std::endl;
            std::cout << "w       switch wireframe/normal display mode" << std::endl;
            std::cout << "k       turn on/off KdTree" << std::endl;
            std::cout << "Esc     quit" << std::endl;
        }

    private:
        /// solvers
        int solver_id;
        std::vector<SolverPtr> solver_list;
        SolverPtr solver;
        CollisionPtr collision;

        /// forces
        ForcePtrContainer forces;
        ForcePtr gravity;

        /// dynamical state for all particles
        DynamicalState DS;
        /// mesh
        GeometryPtr geom;

        /// keyboard selection
        float g;    // gravity constant
        size_t num;  // num of particles
        double Cr;   // coefficient of restitution
        double Cs;   // coefficient of stickness
        bool addParticles;  // flag to decide whether emit particles
        bool wireframe; // flag to decide mesh display mode
        bool onKdTree;  // flage to turn on/off kdtree

        /// default attributes
        float mass;


        //! set default value
        void _init()
        {
            g = 0.48;
            dt = 1.0 / 24.0;
            num = 100;  // init particles number
            Cr = 1.0;   // init elastic collision
            Cs = 1.0;
            mass = 1.0;
        }

        //! emit particles and set dynamical state
        void emitParticles(size_t particle_num)
        {
            size_t nb = DS->nb();
            DS->add(particle_num);
            for (size_t i = 0; i < particle_num; ++i)
            {
                size_t id = nb + i;
                DS->set_id(id, int(id));
                DS->set_pos(id, Vector(0.0, 0.0, 0.0));  // default position
                DS->set_vel(id, Vector((drand48() - 0.5)* 0.5, 0.0, (drand48() - 0.5) * 0.5));  // default velocity
                DS->set_ci(id, Color(0.0, float(drand48() * 0.75 + 0.25), float(drand48() * 0.75 + 0.25), 1.0));    // random color
                DS->set_mass(id, mass);    // default mass
            }
        }

        //! add force to sim
        void addForce()
        {
            gravity = CreateGravity(DS, g); // add gravity
            std::cout << "- Add force " << gravity->Name() << std::endl;
            forces.push_back(gravity);
        }
    };


    pba::PbaThing TriangleCollision(){ return PbaThing( new pba::TriangleCollisionThing() ); }

}

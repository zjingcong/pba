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
            // construct solvers
            solver_list.push_back(new LeapFrogSolver());
            solver_list.push_back(new SixOrderSolver());
            solver = solver_list[LEAP_FROG];
            // construct dynamical state
            DS = CreateDynamicalState("collisionParticles");
            cout << "Construction Complete." << endl;
        }

        ~TriangleCollisionThing()
        {
            delete solver_list[LEAP_FROG];
            delete solver_list[SIX_ORDER];
            delete force;
            delete geom;
        }

        void Init(const std::vector<std::string>& args)
        {
            /// load scene
            // load geometry
            geom = new TriangleGeometry("collisionMesh");
            if (args.size() == 2)   // load .obj file
            {
                std::string scene_file = args[1];
                LoadMesh::LoadObj(scene_file, geom);
            }
            else    // load cube
            { LoadMesh::LoadBox(10, geom); }
            // set geometry color
            for (auto it = geom->get_triangles().begin(); it != geom->get_triangles().end(); ++it)
            {
                TrianglePtr triangle = *it;
                triangle->setColor(Color(float(drand48()), float(drand48()), float(drand48()), 1.0));   // set random colors
            }
            // build geom kdTree
            geom->build_trianglesTree(5);
            cout << "-------------------------------------------" << endl;

            /// init sim
            // add force
            addForce();
            // init dynamical state
            emitParticles(num);
        }

        void Reset()
        {
            // clear dynamical state
            DS.reset();
            DS = CreateDynamicalState("collisionParticles");
            // set default values
            _init();
            // init sim
            emitParticles(num);
        }

        void solve()
        {
            // emit particles
            if (addParticles)
            {
                size_t increase_num = 10;
                emitParticles(increase_num);
                num += increase_num;
                std::cout << "particles number: " << num << std::endl;
            }

            // update force parms
            force->updateParms("g", g);
            // update dynamical state
            // solver->updateDS(dt, DS, force); // no collision
            if (onKdTree)   {solver->updateDSWithCollisionWithKdTree(dt, DS, force, geom, Cr, Cs);}
            else    {solver->updateDSWithCollision(dt, DS, force, geom, Cr, Cs);} // collision

            // solver->updateDSWithCollision(dt, DS, force, geom, Cr, Cs);
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
                { g /= 1.1; std::cout << "gravity constant: " << g << std::endl; break; }
                case 'G':
                { g *= 1.1; std::cout << "gravity constant: " << g << std::endl; break; }

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
                { Cr /= 1.1;    std::cout << "coefficient of restitution Cr: " << Cr << std::endl;  break; }
                case 'C':
                {
                    Cr *= 1.1;
                    if (Cr >= 1.0)  {Cr = 1.0;}
                    std::cout << "coefficient of restitution Cr: " << Cr << std::endl;
                    break;
                }
                case 's':   // Cs
                { Cs /= 1.1;    std::cout << "coefficient of stickiness  Cs: " << Cs << std::endl;  break; }
                case 'S':
                {
                    Cs *= 1.1;
                    if (Cs >= 1.0)  {Cs = 1.0;}
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
            std::cout << "=== PbaThing ===" << endl;
            std::cout << "c/C     reduce/increase collision coefficient of restitution" << endl;
            std::cout << "s/S     reduce/increase collision coefficient of stickiness" << endl;
            std::cout << "g/G     reduce/increase magnitude of gravity" << endl;
            std::cout << "e       start/stop emitting more particles" << endl;
            std::cout << "t/T     reduce/increase animation time step" << endl;
            std::cout << "q       switch solvers between Leap Frog and Sixth Order" << endl;
            std::cout << "w       switch wireframe/normal display mode" << endl;
            std::cout << "Esc     quit" << endl;
        }

    private:
        /// solvers
        int solver_id;
        std::vector<SolverPtr> solver_list;
        SolverPtr solver;

        /// forces
        ForcePtr force;
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
            num = 1;  // init particles number
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
                DS->set_ci(id, Color(float(drand48() * 0.75 + 0.25), float(drand48() * 0.75 + 0.25), float(drand48() * 0.75 + 0.25), 1.0));    // random color
                DS->set_mass(id, mass);    // default mass
            }
        }

        //! add force to sim
        void addForce()
        {
            force = new Gravity(g); // add gravity
            cout << "- Add force " << force->Name() << endl;
        }
    };


    pba::PbaThing TriangleCollision(){ return PbaThing( new pba::TriangleCollisionThing() ); }

}

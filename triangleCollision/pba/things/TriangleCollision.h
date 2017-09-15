//
// Created by jingcoz on 9/13/17.
//

# include "PbaThing.h"
# include "Vector.h"
# include "Color.h"
# include "Solver.h"
# include "Force.h"
# include "DynamicalState.h"

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

// Usage:
//  - collision coefficient of restitution (keys c/C to reduce/increase)
//  - collision coefficient of stickiness (keys s/S to reduce/increase)
//  - number of particles (key e to start/stop emitting more particles)
//  - magnitude of gravity (keys g/G to reduce/increase the magnitude)
//  - timestep size (keys t/T to reduce/increase) - this is already implemented in PbaThingyDingy


namespace pba {

    class TriangleCollisionThing: public PbaThingyDingy
    {
    public:
        TriangleCollisionThing(const std::string nam = "TriangleCollisionThing"):
                PbaThingyDingy(nam),
                solver_id(LEAP_FROG),
                addParticles(false)
        {
            _init();
            solver_list.push_back(new LeapFrogSolver());
            solver_list.push_back(new SixOrderSolver());
            solver = solver_list[LEAP_FROG];
            DS = CreateDynamicalState("collisionParticles");
        }
        ~TriangleCollisionThing()
        {
            delete solver_list[LEAP_FROG];
            delete solver_list[SIX_ORDER];
        }

        void Init(const std::vector<std::string>& args)
        {
            emitParticles(num);
        }

        void Reset()
        {
            DS.reset();
            DS = CreateDynamicalState("collisionParticles");
            _init();
            emitParticles(num);
        }

        void solve()
        {
            // emit particles
            if (addParticles)
            {
                size_t increase_num = 1;
                emitParticles(increase_num);
                num += increase_num;
                std::cout << "particles number: " << num << std::endl;
            }

            // add force
            addForce();
            // set force and ds to solver
            solver->setForce(force);
            solver->setDS(DS);
            // update dynamical state
            solver->updateDS(dt);
        }

        void Display()
        {
            glMatrixMode(GL_MODELVIEW);

            // draw particles
            for (size_t i = 0; i < DS->nb(); ++i)
            {
                glPointSize(3.6f);
                glBegin(GL_POINTS);
                Vector pos = DS->pos(i);
                Color color = DS->ci(i);
                glColor3f(color.red(), color.green(), color.blue());
                glVertex3f(float(pos.X()), float(pos.Y()), float(pos.Z()));
                glEnd();
            }

            glFlush();
        }

        void Keyboard(unsigned char key, int x, int y)
        {
            switch (key)
            {
                // gravity control
                case 'g':
                { g /= 1.1; std::cout << "gravity constant: " << g << std::endl; break; }
                case 'G':
                { g *= 1.1; std::cout << "gravity constant: " << g << std::endl; break; }

                // timestep control
                case 't':
                { dt /= 1.1; std::cout << "time step " << dt << std::endl; break;}
                case 'T':
                { dt *= 1.1; std::cout << "time step " << dt << std::endl; break;}

                // solver switch
                case 'q':
                {
                    solver_id += 1;
                    solver_id = (solver_id % 2);
                    solver = solver_list[solver_id];
                    std::cout << "Current Solver: " << solver->Name() << std::endl;
                    break;
                }

                // particles number control
                case 'e':
                {
                    addParticles = !addParticles;
                    if (addParticles)
                    {
                        std::cout << "START EMITTING" << std::endl;
                    }
                    else
                    {
                        std::cout << "STOP EMITTING" << std::endl;
                    }
                    break;
                }

                // quit
                case 27:
                { exit(0);}

                default:
                    break;
            }
        }

        void Usage()
        {
            std::cout << "=== PbaThing ===\n";
            std::cout << "c/C     reduce/increase collision coefficient of restitution\n";
            std::cout << "s/S     reduce/increase collision coefficient of stickiness\n";
            std::cout << "g/G     reduce/increase magnitude of gravity\n";
            std::cout << "e       start/stop emitting more particles\n";
            std::cout << "t/T     reduce/increase animation time step\n";
            std::cout << "q       switch solvers between Leap Frog and Sixth Order" << endl;
            std::cout << "Esc     quit" << endl;
        }

    private:
        // solver
        int solver_id;
        std::vector<SolverPtr> solver_list;
        SolverPtr solver;
        ForcePtr force;

        // keyboard selection
        float g;    // gravity constant
        float dt;   // timestep
        size_t num;  // num of particles
        float Cr;   // coefficient of restitution
        float Cs;   // coefficient of stickness
        bool addParticles;  // flag to decide whether emit particles

        // default attributes
        float default_mass;

        DynamicalState DS;  // dynamical state for all particles


        // set default value
        void _init()
        {
            g = 0.098;
            dt = float(1.0 / 24.0);
            num = 100;  // init particles number
            Cr = 1.0;
            Cs = 1.0;
            default_mass = 1.0;
        }


        // emit particles and set dynamical state
        void emitParticles(size_t particle_num)
        {
            size_t nb = DS->nb();
            DS->add(particle_num);
            for (size_t i = 0; i < particle_num; ++i)
            {
                size_t id = nb + i;
                DS->set_id(id, int(id));
                DS->set_pos(id, Vector(0.0, 0.0, 0.0));  // default position
                DS->set_vel(id, Vector(drand48() - 0.5, 0.0, drand48() - 0.5));  // default velocity
                DS->set_ci(id, Color(float(drand48() * 0.75 + 0.25), float(drand48() * 0.75 + 0.25), float(drand48() * 0.75 + 0.25), 1.0));    // random color
                DS->set_mass(id, default_mass);    // default mass
            }
        }

        // add force to sim
        void addForce()
        {
            force = new Gravity(g); // add gravity
        }

        // draw cube
        void drawCube()
        {

        }
    };


    pba::PbaThing TriangleCollision(){ return PbaThing( new pba::TriangleCollisionThing() ); }

}

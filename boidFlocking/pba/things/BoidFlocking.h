//
// Created by jingcoz on 10/2/17.
//

# include "PbaThing.h"
# include "Vector.h"
# include "Color.h"
# include "Solver.h"
# include "Force.h"
# include "DynamicalState.h"
# include "Tools.h"
# include "Geometry.h"
# include "Collision.h"

# ifndef PBA_BOIDFLOCKING_H
# define PBA_BOIDFLOCKING_H

# ifdef __APPLE__
# include <OpenGL/gl.h>   // OpenGL itself.
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

    class BoidFlockingThing : public PbaThingyDingy
    {
    public:
        BoidFlockingThing(const std::string nam = "BoidFlockingThing"):
                PbaThingyDingy(nam),
                solver_id(LEAP_FROG),
                display_mode(1),
                onKdTree(false),
                addBoids(false)
        {
            // construct attributes
            _init();
            // construct solvers
            solver_list.push_back(new LeapFrogSolver());
            solver_list.push_back(new SixOrderSolver());
            solver = solver_list[LEAP_FROG];
            // construct dynamical state
            DS = CreateDynamicalState("boidParticles");
            // construct boid system
            boid = new Boid(DS);
            // set boid force
            boidInner = new BoidInnerForce(boid);
            forces.push_back(boidInner);
            cout << "Construction Complete." << endl;
        }

        ~BoidFlockingThing()
        {
            delete solver_list[LEAP_FROG];
            delete solver_list[SIX_ORDER];
            delete boid;
            delete geom;
            for (auto& it: forces)
            {
                delete it;
            }
        }

        void Init(const std::vector<std::string> &args)
        {
            /// load scene
            // load geometry
            geom = new TriangleGeometry("collisionMesh");
            if (args.size() == 2)   // load .obj file
            {
                std::string scene_file = args[1];
                LoadMesh::LoadObj(scene_file, geom);
                geom->build_trianglesTree(5);   // build geom kdTree
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
                triangle->setColor(
                        Color(float(drand48()), float(drand48()), float(drand48()), 1.0));   // set random colors
            }
            cout << "-------------------------------------------" << endl;

            /// init sim
            // init boid system dynamical state
            setBoidDS(boids_num);
            // set boid attributes
            setBoid();
        }

        void Reset()
        {
            // clear dynamical state
            DS.reset();
            DS = CreateDynamicalState("boidParticles");
            // set default values
            _init();
            // init sim
            setBoidDS(boids_num);
            setBoid();
        }

        void solve()
        {
            // add boid
            if (addBoids)
            {
                size_t increase_num = 1;
                setBoidDS(increase_num);
                std::cout << "boid particles number: " << DS->nb() << std::endl;
            }
            // update boid attributes
            setBoid();
            // update dynamical state
            // solver->updateDS(dt, DS, forces); // no collision
            if (onKdTree) { solver->updateDSWithCollisionWithKdTree(dt, DS, forces, geom, 1.0, 1.0); }   // Cr = Cs = 1.0
            else { solver->updateDSWithCollision(dt, DS, forces, geom, 1.0, 1.0); } // collision
        }

        void Display()
        {
            // draw scene
            if (display_mode != 0)  {Draw::DrawTriangles(geom);}
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
                /// boid control
                // Collision avoidance strength
                case 'a':
                {
                    Ka /= 1.1;
                    std::cout << "collision avoidance strength: " << Ka << std::endl;
                    break;
                }
                case 'A':
                {
                    Ka *= 1.1;
                    std::cout << "collision avoidance strength: " << Ka << std::endl;
                    break;
                }
                // Velocity matching strength
                case 'v':
                {
                    Kv /= 1.1;
                    std::cout << "velocity matching strength: " << Kv << std::endl;
                    break;
                }
                case 'V':
                {
                    Kv *= 1.1;
                    std::cout << "velocity matching strength: " << Kv << std::endl;
                    break;
                }
                // Centering strength
                case 'c':
                {
                    Kc /= 1.1;
                    std::cout << "centering strength: " << Kc << std::endl;
                    break;
                }
                case 'C':
                {
                    Kc *= 1.1;
                    std::cout << "centering strength: " << Kc << std::endl;
                    break;
                }
                // Maximum acceleration threshold
                case 'm':
                {
                    accel_max /= 1.1;
                    std::cout << "maximum acceleration threshold: " << accel_max << std::endl;
                    break;
                }
                case 'M':
                {
                    accel_max *= 1.1;
                    std::cout << "maximum acceleration threshold: " << accel_max << std::endl;
                    break;
                }
                // Range
                case 'd':
                {
                    range /= 1.1;
                    std::cout << "range: " << range << std::endl;
                    break;
                }
                case 'D':
                {
                    range *= 1.1;
                    std::cout << "range: " << range << std::endl;
                    break;
                }
                // Range ramp
                case 'y':
                {
                    range_ramp /= 1.1;
                    std::cout << "range ramp: " << range_ramp << std::endl;
                    break;
                }
                case 'Y':
                {
                    range_ramp *= 1.1;
                    std::cout << "range ramp: " << range_ramp << std::endl;
                    break;
                }
                // Field of view
                case 'q':
                {
                    fov /= 1.1;
                    if (fov < 0.0)  {fov = 0.0;}
                    std::cout << "field of view: " << fov << std::endl;
                    break;
                }
                case 'Q':
                {
                    fov *= 1.1;
                    if (fov > 360.0)    {fov = 360.0;}
                    std::cout << "field of view: " << fov << std::endl;
                    break;
                }
                // Peripheral field of view
                case 'p':
                {
                    peripheral_fov /= 1.1;
                    if (peripheral_fov < 0.0)  {peripheral_fov = 0.0;}
                    if (peripheral_fov < fov)   {peripheral_fov = fov;}
                    std::cout << "peripheral field of view: " << peripheral_fov << std::endl;
                    break;
                }
                case 'P':
                {
                    peripheral_fov *= 1.1;
                    if (peripheral_fov > 360.0)    {peripheral_fov = 360.0;}
                    std::cout << "peripheral field of view: " << peripheral_fov << std::endl;
                    break;
                }

                /// solver switch
                case 's':
                {
                    solver_id += 1;
                    solver_id = (solver_id % 2);
                    solver = solver_list[solver_id];
                    std::cout << "Current Solver: " << solver->Name() << std::endl;
                    break;
                }

                /// mesh display mode
                case 'l':
                {
                    display_mode += 1;
                    display_mode = (display_mode % 3);
                    switch (display_mode)
                    {
                        case 0:
                            cout << "Current Display Mode: hide" << endl;
                            break;

                        case 1:
                            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                            cout << "Current Display Mode: normal" << endl;
                            break;

                        case 2:
                            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                            cout << "Current Display Mode: wireframe" << endl;
                            break;

                        default:
                            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                            cout << "Current Display Mode: normal" << endl;
                            break;
                    }
                    break;
                }

                /// kdtree
                case 'k':
                {
                    onKdTree = !onKdTree;
                    if (onKdTree) { std::cout << "TURN ON KDTREE" << std::endl; }
                    else { std::cout << "TURN OFF KDTREE" << std::endl; }
                    break;
                }

                /// add particles
                case 'e':
                {
                    addBoids  = !addBoids;
                    if (addBoids) { std::cout << "START ADDING BOID" << std::endl; }
                    else  {std::cout << "STOP ADDING BOID" << std::endl;}
                    break;
                }

                /// quit
                case 27:
                {
                    exit(0);
                }

                default:
                    break;
            }
        }

        void Usage()
        {
            std::cout << "=== PbaThing ===" << endl;
            std::cout << "a/A     reduce/increase collision avoidance strength" << endl;
            std::cout << "v/V     reduce/increase velocity matching strength" << endl;
            std::cout << "c/C     reduce/increase centering strength" << endl;
            std::cout << "m/M     reduce/increase max acceleration threshold" << endl;
            std::cout << "d/D     reduce/increase range" << endl;
            std::cout << "y/Y     reduce/increase range ramp" << endl;
            std::cout << "q/Q     reduce/increase fov" << endl;
            std::cout << "p/P     reduce/increase peripheral fov" << endl;
            std::cout << "s       switch solvers between Leap Frog and Sixth Order" << endl;
            std::cout << "e       add boid particles" << endl;
            std::cout << "l       switch wireframe/hide/normal display mode" << endl;
            std::cout << "k       turn on/off KdTree" << endl;
            std::cout << "Esc     quit" << endl;
        }

    private:
        /// solvers
        int solver_id;
        std::vector<SolverPtr> solver_list;
        SolverPtr solver;
        /// forces
        ForcePtrContainer forces;
        ForcePtr boidInner;
        /// dynamical state for all particles
        DynamicalState DS;
        /// mesh
        GeometryPtr geom;
        /// boid
        BoidPtr boid;

        /// keyboard selection
        double Ka;   // collision avoidance strength
        double Kv;  // velocity matching strength
        double Kc;  // centering strength
        double accel_max;   // max acceleration threshold
        double range;   // range
        double range_ramp;  // range ramp
        double fov; // field of view
        double peripheral_fov;    // peripheral field of view
        int display_mode;   // 0 - hide, 1 - fill, 2 - line
        bool onKdTree;  // turn on/off kdtree
        bool addBoids;  // flag to add boid particles

        /// default setting
        size_t boids_num;

        //! set default value
        void _init()
        {
            Ka = 0.8;
            Kv = 1.0;
            Kc = 1.2;
            accel_max = 5.0;
            range = 1.0;
            range_ramp = 0.1;
            fov = 360.0;
            peripheral_fov = 360.0;

            boids_num = 11;
        }

        //! emit particles and set dynamical state
        void setBoidDS(size_t num)
        {
            size_t nb = DS->nb();
            DS->add(num);
            for (size_t i = 0; i < num; ++i)
            {
                size_t id = nb + i;
                DS->set_id(id, int(id));
                DS->set_pos(id, 1.5 * Vector((drand48() - 0.5), (drand48() - 0.5), (drand48() - 0.5)));  // random position
                DS->set_vel(id, 0.3 * Vector((drand48() - 0.5), (drand48() - 0.5), (drand48() - 0.5)));  // random velocity
                DS->set_ci(id, Color(float(drand48() * 0.75 + 0.25), float(drand48() * 0.75 + 0.25),
                                     float(drand48() * 0.75 + 0.25), 1.0));    // random color
                DS->set_mass(id, 1.0);    // default mass is 1.0
            }
        }

        void setBoid()
        {
            boid->set_Ka(Ka);
            boid->set_Kv(Kv);
            boid->set_Kc(Kc);
            boid->set_r1(range);
            boid->set_r_ramp(range_ramp);
            boid->set_theta1(fov);
            boid->set_theta2(peripheral_fov);
            boid->set_accel_max(accel_max);
        }
    };


    pba::PbaThing BoidFlocking() { return PbaThing(new pba::BoidFlockingThing()); }
}


# endif //PBA_BOIDFLOCKING_H

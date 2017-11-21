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
# include "Boid.h"

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

    class BoidFlockingThing: public PbaThingyDingy
    {
    public:
        BoidFlockingThing(const std::string nam = "BoidFlockingThing"):
                PbaThingyDingy(nam),
                solver_id(LEAP_FROG),
                display_mode(1),
                onKdTree(false),
                addBoids(false),
                addGuiding(false),
                boids_num(100)
        {
            // construct attributes
            _init();
            // construct solvers
            solver_list.push_back(CreateLeapFrogSolver());
            solver_list.push_back(CreateSixOrderSolver());
            solver = solver_list[LEAP_FROG];
            // construct dynamical state
            DS = CreateDynamicalState("boidParticles");
            // construct boid system
            boid = CreateBoid(DS);
            cout << "Construction Complete." << endl;
        }

        ~BoidFlockingThing() {}

        void Init(const std::vector<std::string> &args)
        {
            /// load scene
            // load geometry
            geom = CreateGeometry("collisionMesh");
            if (args.size() == 2)   // load .obj file
            {
                std::string scene_file = args[1];
                LoadMesh::LoadObj(scene_file, geom);
                geom->build_trianglesTree(5);   // build geom kdTree
            }
            // load cube
            else
            {
                LoadMesh::LoadBox(6, geom);
                geom->build_trianglesTree(0);   // build geom kdTree
            }
            // set geometry color
            for (auto it = geom->get_triangles().begin(); it != geom->get_triangles().end(); ++it)
            {
                TrianglePtr triangle = *it;
                triangle->setColor(
                        Color(float(drand48() * 0.4 + 0.6), 0.5, float(drand48() * 0.4 + 0.6), 1.0));   // set random colors
            }
            cout << "-------------------------------------------" << endl;

            /// init sim
            // init boid system dynamical state
            setBoidDS(boids_num);
            // set boid attributes
            setBoid();
            boid->set_guiding_forces(guidingForces, guidingLocators);
            // set forces
            boidInner = CreateBoidInnerForce(DS, boid);
            guidingSpringForce = CreateSpring(DS, spring_locator, spring_k);
            deflectMagForce = CreateMagneticForce(DS, magnetic_locator, magnetic_B);
            forces.push_back(boidInner);
        }

        void Reset()
        {
            // clear dynamical state
            DS.reset();
            DS = CreateDynamicalState("boidParticles");
            // init sim: boid dynamical state
            setBoidDS(boids_num);
        }

        void solve()
        {
            // clean collision status
            geom->cleanTrianglesCollisionStatus();

            // add boid
            if (addBoids) { setBoidDS(1); std::cout << "boid particles number: " << DS->nb() << std::endl; }

            // update guidingForces
            guidingSpringForce->update_parms("k", spring_k);
            deflectMagForce->update_parms("B", magnetic_B);
            // update boid attributes
            setBoid();

            // update dynamical state
            // solver->updateDS(dt, DS, forces); // no collision
            // Collision: Cr = Cs = 1.0
            if (onKdTree) { solver->updateDSWithCollisionWithKdTree(dt, DS, forces, geom, 1.0, 1.0); }  // kdtree
            else { solver->updateDSWithCollision(dt, DS, forces, geom, 1.0, 1.0); } // no kdtree
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

        void Keyboard(unsigned char key, int x, int y);

        void Usage();

    private:
        /// solvers
        int solver_id;
        std::vector<SolverPtr> solver_list;
        SolverPtr solver;
        /// forces
        ForcePtr boidInner;
        ForcePtrContainer forces;
        /// boid guiding
        ForcePtr guidingSpringForce;
        Vector spring_locator;
        ForcePtr deflectMagForce;
        Vector magnetic_locator;
        ForcePtrContainer guidingForces;
        std::vector<Vector> guidingLocators;
        /// dynamical state for all particles
        DynamicalState DS;
        /// mesh
        GeometryPtr geom;
        /// boid
        BoidPtr boid;

        /// keyboard selection
        double Ka;              // collision avoidance strength
        double Kv;              // velocity matching strength
        double Kc;              // centering strength
        double accel_max;       // max acceleration threshold
        double range;           // range
        double range_ramp;      // range ramp
        double fov;             // field of view
        double fov_ramp;        // field of view ramp
        int display_mode;       // 0 - hide, 1 - fill, 2 - line
        bool onKdTree;          // turn on/off kdtree mode for geometry
        bool addBoids;          // flag to add boid particles
        bool addGuiding;        // flag to add guiding force
        bool addMag;            // flag to add magnetic force in order to deflect boids
        float spring_k;         // spring force k
        float magnetic_B;       // magnetic force B

        /// default setting
        size_t boids_num;

        //! set default value
        void _init()
        {
            /// boid
            Ka = 0.3;
            Kv = 23;
            Kc = 16;
            accel_max = 20;
            range = 0.25;
            range_ramp = 0.8;
            fov = 160.0;
            fov_ramp = 70.0;
            /// boid guiding
            spring_k = 0.2;
            magnetic_B = 20.0;
            spring_locator = Vector(-3.0, -3.0, 0.0);
            magnetic_locator = Vector(3.0, -3.0, 0.0);
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
                DS->set_pos(id, 1 * Vector((drand48() - 0.5), (drand48() - 0.5), (drand48() - 0.5)));  // random position
                DS->set_vel(id, 5 * Vector((drand48() - 0.5), (drand48() - 0.5), (drand48() - 0.5)));  // random velocity
                DS->set_ci(id, Color(0.0, float(drand48() * 0.75 + 0.25),
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
            boid->set_theta_ramp(fov_ramp);
            boid->set_accel_max(accel_max);
        }

        void guidingControl()
        {
            // clean guide container
            guidingForces.clear();
            guidingLocators.clear();

            // set guiding force container
            if (addGuiding) {guidingForces.push_back(guidingSpringForce);   guidingLocators.push_back(spring_locator);}
            if (addMag) {guidingForces.push_back(deflectMagForce);  guidingLocators.push_back(magnetic_locator);}
        }
    };

    //! create pbathing
    pba::PbaThing BoidFlocking() { return PbaThing(new pba::BoidFlockingThing()); }


    //! ---------------------------------------- Keyboard and Usage ---------------------------------------------------

    void BoidFlockingThing::Usage()
    {
        std::cout << "=== PbaThing ===" << endl;
        std::cout << "a/A     reduce/increase collision avoidance strength" << endl;
        std::cout << "v/V     reduce/increase velocity matching strength" << endl;
        std::cout << "c/C     reduce/increase centering strength" << endl;
        std::cout << "m/M     reduce/increase max acceleration threshold" << endl;
        std::cout << "d/D     reduce/increase range" << endl;
        std::cout << "y/Y     reduce/increase range ramp" << endl;
        std::cout << "q/Q     reduce/increase fov" << endl;
        std::cout << "-----------------------" << endl;
        std::cout << "---- More Controls ----" << endl;
        std::cout << "-----------------------" << endl;
        std::cout << "p/P     reduce/increase fov ramp" << endl;
        std::cout << "g       turn on/off guiding spring force" << endl;
        std::cout << "k/K     reduce/increase spring force k" << endl;
        std::cout << "f       turn on/off magnetic force" << endl;
        std::cout << "b/B     reduce/increase magnetic force B" << endl;
        std::cout << "s       switch solvers between Leap Frog and Sixth Order" << endl;
        std::cout << "e       add boid particles" << endl;
        std::cout << "l       switch wireframe/hide/normal display mode" << endl;
        std::cout << "t       turn on/off KdTree" << endl;
        std::cout << "Esc     quit" << endl;
    }

    void BoidFlockingThing::Keyboard(unsigned char key, int x, int y)
    {
        switch (key)
        {
            /// boid control
            // Collision avoidance strength
            case 'a': { Ka /= 1.1; std::cout << "collision avoidance strength: " << Ka << std::endl; break; }
            case 'A': { Ka *= 1.1; std::cout << "collision avoidance strength: " << Ka << std::endl; break; }
                // Velocity matching strength
            case 'v': { Kv /= 1.1; std::cout << "velocity matching strength: " << Kv << std::endl; break; }
            case 'V': { Kv *= 1.1; std::cout << "velocity matching strength: " << Kv << std::endl; break; }
                // Centering strength
            case 'c': { Kc /= 1.1; std::cout << "centering strength: " << Kc << std::endl; break; }
            case 'C': { Kc *= 1.1; std::cout << "centering strength: " << Kc << std::endl; break; }
                // Maximum acceleration threshold
            case 'm': { accel_max /= 1.1; std::cout << "maximum acceleration threshold: " << accel_max << std::endl; break; }
            case 'M': { accel_max *= 1.1; std::cout << "maximum acceleration threshold: " << accel_max << std::endl; break; }
                // Range
            case 'd': { range /= 1.1; std::cout << "range: " << range << std::endl; break; }
            case 'D': { range *= 1.1; std::cout << "range: " << range << std::endl; break; }
                // Range ramp
            case 'y': { range_ramp /= 1.1; std::cout << "range ramp: " << range_ramp << std::endl; break; }
            case 'Y': { range_ramp *= 1.1; std::cout << "range ramp: " << range_ramp << std::endl; break; }
                // Field of view
            case 'q': { fov /= 1.1; std::cout << "field of view: " << fov << std::endl; break; }
            case 'Q': { fov *= 1.1; std::cout << "field of view: " << fov << std::endl; break; }
                // Peripheral field of view
            case 'p': { fov_ramp /= 1.1; std::cout << "peripheral field of view: " << fov_ramp << std::endl; break; }
            case 'P': { fov_ramp *= 1.1; std::cout << "peripheral field of view: " << fov_ramp << std::endl; break; }
                /// force control
                // spring force
            case 'g':
            {
                addGuiding = !addGuiding;
                if (addGuiding) {std::cout << "TURN ON GUIDING" << endl;}
                else    {std::cout << "TURN OFF GUIDING" << endl;}
                guidingControl();
                break;
            }
            case 'k': { spring_k /= 1.1; std::cout << "spring k: " << spring_k << std::endl; break; }
            case 'K': { spring_k *= 1.1; std::cout << "spring k: " << spring_k << std::endl; break; }
                // magnetic force
            case 'f':
            {
                addMag = !addMag;
                if (addMag) {std::cout << "TURN ON DEFLECTION" << endl;}
                else    {std::cout << "TURN OFF DEFLECTION" << endl;}
                guidingControl();
                break;
            }
            case 'b': { magnetic_B /= 1.1; std::cout << "magnetic B: " << magnetic_B << std::endl; break; }
            case 'B': { magnetic_B *= 1.1; std::cout << "magnetic B: " << magnetic_B << std::endl; break; }

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
            case 't':
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
            case 27: { exit(0); }

            default:
                break;
        }
    }
}


# endif //PBA_BOIDFLOCKING_H

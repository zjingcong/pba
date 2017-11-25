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
                kdtree_level(7),
                spheres_num(10),
                solver_id(LEAP_FROG),
                g(0.48),
                Cr(1.0),
                Cs(1.0),
                on_KdTree(false),
                emit_spheres(false),
                wireframe(false)
        {
            dt = 1.0/24;
            geom = CreateGeometry("wellGeom");
            sphereDS = CreateSphereState("sphereDS");
            collision = CreateSphereCollision(sphereDS);
            solver_list.push_back(CreateLeapFrogSolver());
            solver_list.push_back(CreateSixOrderSolver());
            solver = solver_list[solver_id];
        }

        ~SpheresThing() {}

        void Init(const std::vector<std::string> &args)
        {
            /// load geometry
            LoadMesh::LoadObj("../../models/well.obj", geom);
            // set geometry color
            for (auto &it: geom->get_triangles())
            {
                it->setColor(Color(float(drand48() * 0.1 + 0.35), float(drand48() * 0.1 + 0.35), float(drand48() * 0.1 + 0.35), 1.0));
            }
            geom->build_trianglesTree(kdtree_level);   // build geom kdTree
            std::cout << "-------------------------------------------" << std::endl;

            /// load spheres
            add_spheres(spheres_num);

            /// load forces
            gravity = CreateGravity(sphereDS, g); // add gravity
            std::cout << " - Add force " << gravity->Name() << std::endl;
            forces.push_back(gravity);

            // set collision
            collision->init();
            collision->set_geom(geom);
            collision->set_Cr(Cr);
            collision->set_Cs(Cs);
            solver_list[LEAP_FROG]->setCollision(collision);
            solver_list[SIX_ORDER]->setCollision(collision);
            std::cout << "-------------------------------------------" << std::endl;
        }

        void Reset()
        {
            geom->cleanTrianglesCollisionStatus();
            sphereDS->clear();
            add_spheres(spheres_num);
            print_atts();
        }

        void solve()
        {
            // clean collision status
            geom->cleanTrianglesCollisionStatus();
            sphereDS->clean_collision_flags();
            // add spheres
            if (emit_spheres)   {add_spheres(1);    collision->init();}
            // update dynamical state
            if (on_KdTree)   {solver->updateDSWithCollisionWithKdTree(dt, sphereDS, forces);}
            else    {solver->updateDSWithCollision(dt, sphereDS, forces);}
        }

        void Display()
        {
            Draw::DrawTriangles(geom, true);
            draw_spheres();
            glFlush();
        }

        void Keyboard(unsigned char key, int x, int y);

        void Usage();

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
        size_t spheres_num;
        int solver_id;
        float g;
        float Cr;
        float Cs;
        bool on_KdTree;
        bool emit_spheres;
        bool wireframe;

        void add_spheres(size_t sphere_num)
        {
            size_t nb = sphereDS->nb();
            sphereDS->add(sphere_num);
            for (size_t i = 0; i < sphere_num; ++i)
            {
                size_t id = nb + i;
                sphereDS->set_id(id, int(id));
                sphereDS->set_radius(id, 0.1 * (drand48() + 1.2));  // random radius
                sphereDS->set_pos(id, Vector((drand48() - 0.5), drand48(), (drand48() - 0.5)));  // random position
                sphereDS->set_vel(id, 0.8 * Vector((drand48() - 0.5), 0.0, (drand48() - 0.5)));  // random velocity
                sphereDS->set_ci(id, Color(0.0, float(drand48() * 0.5 + 0.5), float(drand48() * 0.5 + 0.5), 1.0));    // random color
            }
            std::cout << "current spheres number: " << sphereDS->nb() << std::endl;
        }

        void draw_spheres()
        {
            for (size_t i = 0; i < sphereDS->nb(); ++i)
            {
                float radius = sphereDS->radius(i);
                Vector pos = sphereDS->pos(i);
                Color color = sphereDS->ci(i);
                if (sphereDS->isCollision(i) == 1)  {color = Color(1.0, 1.0, 1.0, 1.0);}

                glColor3f(color.X(), color.Y(), color.Z());
                glPushMatrix ();
                glTranslatef(pos.X(), pos.Y(), pos.Z());
                glutSolidSphere(radius, 20, 20);
                glPopMatrix ();
            }
        }

        void print_atts()
        {
            std::cout << "dt: " << dt << std::endl;
            std::cout << "Cr: " << Cr << std::endl;
            std::cout << "Cs: " << Cs << std::endl;
            std::cout << "g:  " << g << std::endl;
        }
    };

    pba::PbaThing Spheres() { return PbaThing(new pba::SpheresThing()); }


    void SpheresThing::Keyboard(unsigned char key, int x, int y)
    {
        PbaThingyDingy::Keyboard(key, x, y);
        switch (key)
        {
            /// gravity control
            case 'g':
            { g /= 1.1; gravity->update_parms("g", g);  std::cout << "gravity constant: " << g << std::endl; break; }
            case 'G':
            { g *= 1.1; gravity->update_parms("g", g);  std::cout << "gravity constant: " << g << std::endl; break; }

            /// collision control
            // Cr
            case 'c':
            { Cr /= 1.1;
                collision->set_Cr(Cr);   std::cout << "Cr: " << Cr << std::endl; break; }
            case 'C':
            { Cr *= 1.1;    Cr = (Cr > 1.0) ? float(1.0): Cr;
                collision->set_Cr(Cr);    std::cout << "Cr: " << Cr << std::endl; break; }
                // Cs
            case 's':
            { Cs /= 1.1;
                collision->set_Cs(Cs);   std::cout << "Cs: " << Cs << std::endl; break; }
            case 'S':
            { Cs *= 1.1;    Cs = (Cs > 1.0) ? float(1.0): Cs;
                collision->set_Cs(Cs);   std::cout << "Cs: " << Cs << std::endl; break; }

            /// kdtree
            case 'a':
            {
                on_KdTree = !on_KdTree;
                if (on_KdTree)   {std::cout << "TURN ON Kdtree" << std::endl;}
                else    {std::cout << "TURN OFF Kdtree" << std::endl;}
                break;
            }

            /// solver switch
            case 'q':
            {
                solver_id = ((solver_id + 1) % 2);
                solver = solver_list[solver_id];
                std::cout << "Current Solver: " << solver->Name() << std::endl;
                break;
            }

            /// add spheres
            case 'e':
            {
                emit_spheres = !emit_spheres;
                if (emit_spheres) { std::cout << "START EMITTING" << std::endl; }
                else { std::cout << "STOP EMITTING" << std::endl; }
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

            /// quit
            case 27: { exit(0); }

            default:
                break;
        }
    }

    void SpheresThing::Usage()
    {
        PbaThingyDingy::Usage();
        std::cout << "c/C          reduce/increase Cr" << std::endl;
        std::cout << "s/S          reduce/increase Cs" << std::endl;
        std::cout << "g/G          reduce/increase gravity constant" << std::endl;
        std::cout << "a            turn on/off kdtree" << std::endl;
        std::cout << "q            switch solver between Leap Frog Solver and Sixth Order Solver" << std::endl;
        std::cout << "e            add spheres" << std::endl;
        std::cout << "w            switch wireframe/normal display mode" << std::endl;
        std::cout << "Esc          quit" << std::endl;
    }
}

#endif //PBA_SPHERESTHING_H

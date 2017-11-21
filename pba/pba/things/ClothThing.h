//
// Created by jingcoz on 10/26/17.
//

#ifndef PBA_CLOTHTHING_H
#define PBA_CLOTHTHING_H

# include "PbaThing.h"
# include "Tools.h"
# include <algorithm>
# include "SBD.h"
# include "Solver.h"
# include "SoftBodyState.h"

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

    class ClothInHoleThing: public PbaThingyDingy
    {
    public:
        ClothInHoleThing(const std::string nam = "ClothInHoleThing"):
                PbaThingyDingy(nam),
                size(5.4),
                plane_div(7),
                cloth_div(20),
                hole_division(1),
                time_step(0),
                kdtree_level(3),
                g(9.8),
                Ks(70),
                Kf(5),
                As(80),
                Af(4),
                Cr(0.1),
                Cs(1.0),
                substep(1),
                onKdtree(true)
        {
            dt = 1.0/45;
            // cloth
            SB = CreateSoftBodyState("ClothStateData");
            // forces
            gravity = CreateGravity(SB, g);
            clothInner = CreateSoftBodyInnerForce(SB);
            forces.push_back(gravity);
            forces.push_back(clothInner);
            // solvers
            // solver = CreateLeapFrogSolver(); // leap frog solver
            solver = CreateSixOrderSolver();    // sixth order solver
            subSolver = CreateSubSolver();
            subSolver->setSolver(solver);
            subSolver->setSubstep(substep);
        }
        ~ClothInHoleThing() {}

        void Init(const std::vector<std::string>& args)
        {
            /// load preset attributes
            if (args[1] == "0")   {preset_0();}
            if (args[1] == "1")   {preset_1();}

            /// load plane with hole
            // create plane
            geom = CreateGeometry("collisionPlane");
            LoadMesh::LoadPlane(Vector(0.0, -2.0, 0.0), size + 0.0000000001, plane_div, geom);
            // create hole
            createHole(hole_division);
            geom->build_trianglesTree(kdtree_level);   // build geom kdTree
            // set random colors
            for (auto& it: geom->get_triangles())
            { it->setColor(Color(0.55, 0.55, 0.55, 1.0));}
            std::cout << "Hole size: " << size * hole_division / plane_div << std::endl;

            /// load cloth
            SB->update_parms("Ks", Ks);
            SB->update_parms("Kf", Kf);
            SB->update_parms("As", As);
            SB->update_parms("Af", Af);
            createGridPlane(SB, Vector(0.0, -1.5, 0.0), size, cloth_div);
            if (std::find(args.begin(), args.end(), "p") != args.end())
            {
                createConnectedPairs(SB, cloth_div);
                std::cout << "connected pairs: " << SB->get_connectedPairs().size() << std::endl;
            }
            if (std::find(args.begin(), args.end(), "a") != args.end())
            {
                createTriangleAreas(SB, cloth_div);
                std::cout << "triangle areas: " << SB->get_triangleAreas().size() << std::endl;
            }

            std::cout << "-------------------------------------------" << std::endl;
        }

        void Reset()
        {
            time_step = 0;
            geom->cleanTrianglesCollisionStatus();
            SB->Reset();
            printAtt();
        }

        void solve()
        {
            time_step++;
            // clean collision status
            geom->cleanTrianglesCollisionStatus();
            // solve
            SB->Update();
            // subsolver
            if (!onKdtree)  {subSolver->updateDSWithCollision(dt, SB, forces, geom, Cr, Cs);}
            else {subSolver->updateDSWithCollisionWithKdtree(dt, SB, forces, geom, Cr, Cs);}
        }

        void Display()
        {
            /// draw plane
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            Draw::DrawTriangles(geom, false);
            /// draw cloth
            // for viz, draw vertices/connected pairs/triangle areas separately
            // draw vertices
            drawVerts();
            // draw connected pairs
            drawPairs();
            // draw triangle areas
            drawAreas();
            glFlush();
        }

        void Keyboard(unsigned char key, int x, int y);
        void Usage();

    private:
        // cloth
        SoftBodyState SB;
        double size;
        // collision plane
        GeometryPtr geom;
        // forces
        ForcePtrContainer forces;
        ForcePtr gravity;
        ForcePtr clothInner;
        // solver
        SolverPtr solver;
        SubSolverPtr subSolver;

        int plane_div;
        int cloth_div;
        int hole_division;
        int time_step;
        int kdtree_level;

        /// keyboard controls
        float g;    // gravity constant
        float Ks;
        float Kf;
        float As;
        float Af;
        float Cr;
        float Cs;
        int substep;
        bool onKdtree;

        void printAtt()
        {
            std::cout << "gravity constant: " << g << std::endl;
            std::cout << "Ks: " << Ks << std::endl;
            std::cout << "Kf: " << Kf << std::endl;
            std::cout << "As: " << As << std::endl;
            std::cout << "Af: " << Af << std::endl;
            std::cout << "Cr: " << Cr << std::endl;
            std::cout << "Cs: " << Cs << std::endl;
            std::cout << "substep: " << substep << std::endl;
            std::cout << "timestep: " << dt << std::endl;
        }

        /////////////////////////////////// build plane ///////////////////////////////////
        void createHole(const int& division_num)
        {
            int low = (plane_div - division_num) / 2;
            int up = (plane_div + division_num) / 2;
            for (int i = low; i < up; ++i)
            {
                for (int j = low; j < up; ++j)
                {
                    int id = i * plane_div + j;
                    geom->get_triangles()[id * 2] = nullptr;
                    geom->get_triangles()[id * 2 + 1] = nullptr;
                }
            }

            geom->get_triangles().erase(std::remove(geom->get_triangles().begin(), geom->get_triangles().end(), nullptr), geom->get_triangles().end());
        }

        /////////////////////////////////// build cloth ///////////////////////////////////
        void createGridPlane(SoftBodyState sb, const Vector& center, const double &size, const int &division)
        {
            std::vector<Vector> verts;
            double grid_size = size / division;
            double l_x = center.X() - size * 0.5;
            double l_z = center.Z() - size * 0.5;
            for (int m = 0; m <= division; ++m)  // x
            {
                for (int n = 0; n <= division; ++n)  // z
                {
                    Vector p = Vector(l_x + m * grid_size, center.Y(), l_z + n * grid_size);
                    verts.push_back(p);
                }
            }
            SB->Init(verts);
        }

        void createConnectedPairs(SoftBodyState sb, const int& division)
        {
            for (int m = 0; m <= division; ++m)  // x
            {
                for (int n = 0; n <= division; ++n)  // z
                {
                    size_t id = m + n * (division + 1);
                    /// create connected pairs
                    // add horizontal and vertical pairs
                    if (m != division) { sb->add_softEdges(id, id + 1); }
                    if (n != division) { sb->add_softEdges(id, id + division + 1); }
                    // add diagonal pairs
//                    if (m != division && n != division) { sb->add_softEdges(id, id + division + 2); }
                }
            }
        }

        void createTriangleAreas(SoftBodyState sb, const int& division)
        {
            for (int m = 0; m <= division; ++m)  // x
            {
                for (int n = 0; n <= division; ++n)  // z
                {
                    size_t id = m + n * (division + 1);
                    /// create triangle areas
                    if (m != division && n != division)
                    {
                        // left upper
                        sb->add_softTriangles(id, id + 1, id + division + 1);
                        // right upper
                        sb->add_softTriangles(id, id + 1, id + division + 2);
                        // left lower
                        sb->add_softTriangles(id, id + division + 1, id + division + 2);
                        // right lower
                        sb->add_softTriangles(id + 1, id + division + 1, id + division + 2);
                    }
                }
            }
        }

        /////////////////////////////////// visualizer ///////////////////////////////////
        void drawVerts()
        {
            glPointSize(3.0f);
            glBegin(GL_POINTS);
            glColor3f(0.5, 0.8, 0.8);   // verts color
            for (size_t i = 0; i < SB->nb(); ++i)
            {
                Vector pos = SB->pos(i);
                glVertex3f(float(pos.X()), float(pos.Y()), float(pos.Z()));
            }
            glEnd();
        }

        void drawPairs()
        {
            glLineWidth(1.2f);
            glBegin(GL_LINES);
            glColor3f(1.0, 1.0, 0.5);   // connected pairs color
            for (auto& it: SB->get_connectedPairs())
            {
                size_t i = it.i;
                size_t j = it.j;
                glVertex3f(float(SB->pos(i).X()), float(SB->pos(i).Y()), float(SB->pos(i).Z()));
                glVertex3f(float(SB->pos(j).X()), float(SB->pos(j).Y()), float(SB->pos(j).Z()));
            }
            glEnd();
        }

        void drawAreas()
        {
            glLineWidth(1.0f);
            glPolygonMode( GL_FRONT_AND_BACK, GL_LINE);
            glBegin(GL_TRIANGLES);
            glColor3f(0.5, 1.0, 0.8);   // triangle areas color
            for (auto& it: SB->get_triangleAreas())
            {
                size_t i = it.i;
                size_t j = it.j;
                size_t k = it.k;
                glVertex3f(float(SB->pos(i).X()), float(SB->pos(i).Y()), float(SB->pos(i).Z()));
                glVertex3f(float(SB->pos(j).X()), float(SB->pos(j).Y()), float(SB->pos(j).Z()));
                glVertex3f(float(SB->pos(k).X()), float(SB->pos(k).Y()), float(SB->pos(k).Z()));
            }
            glEnd();
        }

        /////////////////////////////////// preset values ///////////////////////////////////
        void preset_0()
        {
            plane_div = 11;
            cloth_div = 24;
            hole_division = 1;
            time_step = 0;
            kdtree_level = 5;
            Ks = 100;
            Kf = 9.5;
            As = 0.0;
            Af = 0.0;
            dt = 1.0/56;
        }

        void preset_1()
        {
            plane_div = 19;
            cloth_div = 26;
            hole_division = 1;
            time_step = 0;
            kdtree_level = 6;
            Ks = 80;
            Kf = 8;
            As = 80;
            Af = 4;
            dt = 1.0/55;
        }
    };

    pba::PbaThing ClothInHole() { return PbaThing(new pba::ClothInHoleThing()); }


    //! ======================================= Keyboards and Usage ===================================================

    void ClothInHoleThing::Keyboard(unsigned char key, int x, int y)
    {
        if (key >= '1' && key <= '9')
        {
            substep = int(key) - 48;
            std::cout << "Set sub step: " << substep << std::endl;
            subSolver->setSubstep(substep);
            return;
        }

        PbaThingyDingy::Keyboard(key, x, y);

        switch (key)
        {
            /// gravity control
            case 'g':
            { g /= 1.1; gravity->update_parms("g", g);  std::cout << "gravity constant: " << g << std::endl; break; }
            case 'G':
            { g *= 1.1; gravity->update_parms("g", g);  std::cout << "gravity constant: " << g << std::endl; break; }

            /// cloth force control
            // Ks
            case 's':
            { Ks /= 1.1; SB->update_parms("Ks", Ks);  std::cout << "Ks: " << Ks << std::endl; break; }
            case 'S':
            { Ks *= 1.1; SB->update_parms("Ks", Ks);  std::cout << "Ks: " << Ks << std::endl; break; }
            // Kf
            case 'k':
            { Kf /= 1.1; SB->update_parms("Kf", Kf);  std::cout << "Kf: " << Kf << std::endl; break; }
            case 'K':
            { Kf *= 1.1; SB->update_parms("Kf", Kf);  std::cout << "Kf: " << Kf << std::endl; break; }
            // As
            case 'z':
            { As /= 1.1; SB->update_parms("As", As);  std::cout << "As: " << As << std::endl; break; }
            case 'Z':
            { As *= 1.1; SB->update_parms("As", As);  std::cout << "As: " << As << std::endl; break; }
            // Af
            case 'v':
            { Af /= 1.1; SB->update_parms("Af", Af);  std::cout << "Af: " << Af << std::endl; break; }
            case 'V':
            { Af *= 1.1; SB->update_parms("Af", Af);  std::cout << "Af: " << Af << std::endl; break; }

            /// collision control
            // Cr
            case 'c':
            { Cr /= 1.1;    std::cout << "Cr: " << Cr << std::endl; break; }
            case 'C':
            { Cr *= 1.1;    Cr = (Cr > 1.0) ? float(1.0): Cr;  std::cout << "Cr: " << Cr << std::endl; break; }
            // Cs
            case 'x':
            { Cs /= 1.1;    std::cout << "Cs: " << Cs << std::endl; break; }
            case 'X':
            { Cs *= 1.1;    Cs = (Cs > 1.0) ? float(1.0): Cs;   std::cout << "Cs: " << Cs << std::endl; break; }

            /// print current time step
            case ' ':
            { std::cout << "Current time step: " << time_step << std::endl; break;}

            /// kdtree
            case 'a':
            {
                onKdtree = !onKdtree;
                if (onKdtree)   {std::cout << "TURN ON Kdtree" << std::endl;}
                else    {std::cout << "TURN OFF Kdtree" << std::endl;}
                break;
            }

            /// quit
            case 27: { exit(0); }

            default:
                break;
        }
    }

    void ClothInHoleThing::Usage()
    {
        PbaThingyDingy::Usage();
        std::cout << "s/S          reduce/increase Ks" << std::endl;
        std::cout << "k/K          reduce/increase Kf" << std::endl;
        std::cout << "z/Z          reduce/increase As" << std::endl;
        std::cout << "v/V          reduce/increase Af" << std::endl;
        std::cout << "c/C          reduce/increase Cr" << std::endl;
        std::cout << "x/X          reduce/increase Cs" << std::endl;
        std::cout << "g/G          reduce/increase gravity constant" << std::endl;
        std::cout << "a            turn on/off kdtree" << std::endl;
        std::cout << "1-9          specify sub step" << std::endl;
        std::cout << "Esc          quit" << std::endl;
    }

}

#endif //PBA_CLOTHTHING_H

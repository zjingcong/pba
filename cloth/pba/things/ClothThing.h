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
                plane_div(11),     // be odd number
                cloth_div(50),
                hole_division(3),
                time_step(0),
                kdtree_level(6),
                g(0.98),
                Ks(100),
                Kf(100),
                Cr(1.0),
                Cs(1.0),
                substep(1),
                onKdtree(false)
        {
            dt = 1.0/240;
            // cloth
            SB = CreateSoftBodyState("ClothStateData");
            SB->update_parms("Ks", Ks);
            SB->update_parms("Kf", Kf);
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
            /// load plane with hole
            // create plane
            geom = CreateGeometry("collisionPlane");
            LoadMesh::LoadPlane(Vector(0.0, -2.0, 0.0), size + 0.000001, plane_div, geom);
            // create hole
            createHole(hole_division);
            geom->build_trianglesTree(kdtree_level);   // build geom kdTree
            // set random colors
            for (auto& it: geom->get_triangles())
            { it->setColor(Color(float(drand48() * 0.4 + 0.6), 0.5, float(drand48() * 0.4 + 0.6), 1.0));}

            /// load cloth
            std::vector<std::pair<size_t, size_t>> edge_pairs;
            createGridPlane(Vector(0.0, -1.8, 0.0), size, cloth_div, verts, edge_pairs);
            SB->Init(verts);
            SB->set_softEdges(edge_pairs);
            std::cout << "connected pairs: " << edge_pairs.size() << std::endl;

            std::cout << "-------------------------------------------" << std::endl;
        }

        void Reset()
        {
            time_step = 0;
            geom->cleanTrianglesCollisionStatus();
            SB->Reset(verts);
        }

        void solve()
        {
            time_step++;
            // clean collision status
            geom->cleanTrianglesCollisionStatus();
            // solve
            SB->update_innerForce();
            // if (!onKdtree)  {solver->updateDSWithCollision(dt, SB, forces, geom, Cr, Cs);}
            // else {solver->updateDSWithCollisionWithKdTree(dt, SB, forces, geom, Cr, Cs);}
            // subsolver
            if (!onKdtree)  {subSolver->updateDSWithCollision(dt, SB, forces, geom, Cr, Cs);}
            else {subSolver->updateDSWithCollisionWithKdtree(dt, SB, forces, geom, Cr, Cs);}
        }

        void Display()
        {
            /// draw plane
            Draw::DrawTriangles(geom, false);

            /// draw cloth
            // draw vertices
            glPointSize(3.0f);
            glBegin(GL_POINTS);
            glColor3f(0.5, 0.8, 0.8);   // vert color
            for (size_t i = 0; i < SB->nb(); ++i)
            {
                Vector pos = SB->pos(i);
                glVertex3f(float(pos.X()), float(pos.Y()), float(pos.Z()));
            }
            glEnd();
            // draw connected pairs
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

        void Usage()
        {
            PbaThingyDingy::Usage();
            std::cout << "s/S          reduce/increase Ks" << std::endl;
            std::cout << "k/K          reduce/increase Kf" << std::endl;
            std::cout << "c/C          reduce/increase Cr" << std::endl;
            std::cout << "x/X          reduce/increase Cs" << std::endl;
            std::cout << "g/G          reduce/increase gravity constant" << std::endl;
            std::cout << "a            turn on/off kdtree" << std::endl;
            std::cout << "1-9          specify sub step" << std::endl;
            std::cout << "Esc          quit" << std::endl;
        }

    private:
        // cloth
        SoftBodyState SB;
        double size;
        std::vector<Vector> verts;
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
        float Cr;
        float Cs;
        int substep;
        bool onKdtree;

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

        void createGridPlane(const Vector& center, const double &size, const int &division,
                             std::vector<Vector>& verts,
                             std::vector< std::pair<size_t, size_t> > &edge_pairs)
        {
            double grid_size = size / division;
            double l_x = center.X() - size * 0.5;
            double l_z = center.Z() - size * 0.5;
            for (int m = 0; m <= division; ++m)  // x
            {
                for (int n = 0; n <= division; ++n)  // z
                {
                    Vector p = Vector(l_x + m * grid_size, center.Y(), l_z + n * grid_size);
                    verts.push_back(p);
                    size_t id = m + n * (division + 1);

                    /// create connected pairs
                    // add horizontal and vertical pairs
                    if (m != division)
                    {
                        std::pair<size_t , size_t > pair_h = std::make_pair(id, id + 1);
                        edge_pairs.push_back(pair_h);
                    }
                    if (n != division)
                    {
                        std::pair<size_t , size_t > pair_v = std::make_pair(id, id + division + 1);
                        edge_pairs.push_back(pair_v);
                    }
                    // add diagonal pairs
                    if (m != division && n != division)
                    {
                        std::pair<size_t , size_t > pair_v = std::make_pair(id, id + division + 2);
                        edge_pairs.push_back(pair_v);
                    }
                }
            }
        }
    };

    pba::PbaThing ClothInHole() { return PbaThing(new pba::ClothInHoleThing()); }

}

#endif //PBA_CLOTHTHING_H

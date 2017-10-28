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

# ifdef __APPLE__
# include <OpenGL/gl.h>   // OpenGL itself.
  #include <OpenGL/glu.h>  // GLU support library.
  #include <GLUT/glut.h>
# else
# include <GL/gl.h>   // OpenGL itself.
# include <GL/glu.h>  // GLU support library.
# include <GL/glut.h> // GLUT support library.
#include <SoftBodyState.h>

# endif

namespace pba
{

    class ClothInHoleThing: public PbaThingyDingy
    {
    public:
        ClothInHoleThing(const std::string nam = "ClothInHoleThing"):
                PbaThingyDingy(nam),
                size(4.0),
                plane_div(11),     // be odd number
                cloth_div(10),
                g(0.98)
        {
            SB = CreateSoftBodyState("ClothStateData");
            dt = 1.0/24;
            // forces
            gravity = CreateGravity(SB, g);
            clothInner = CreateSoftBodyInnerForce(SB);
            forces.push_back(gravity);
            forces.push_back(clothInner);
            // solvers
            solver = CreateLeapFrogSolver();
        }
        ~ClothInHoleThing() {}

        void Init(const std::vector<std::string>& args)
        {
            /// load plane with hole
            // create plane
            geom = CreateGeometry("collisionPlane");
            LoadMesh::LoadPlane(Vector(0.0, -2.0, 0.0), size, plane_div, geom);
            // create hole
            createHole(3);
            // set random colors
            for (auto& it: geom->get_triangles())
            { it->setColor(Color(float(drand48() * 0.4 + 0.6), 0.5, float(drand48() * 0.4 + 0.6), 1.0));}

            /// load cloth
            std::vector<Vector> verts;
            std::vector<std::pair<size_t, size_t >> edge_pairs;
            createGridPlane(Vector(0.0, 2.0, 0.0), size, cloth_div, verts, edge_pairs);
            SB->Init(verts);
            SB->set_softEdges(edge_pairs);

            std::cout << "-------------------------------------------" << std::endl;
        }

        void Reset()
        {

        }

        void solve()
        {
            solver->updateDS(dt, SB, forces);
        }

        void Display()
        {
            /// draw plane
            Draw::DrawTriangles(geom);

            /// draw cloth
            // draw vertices
            glPointSize(3.0f);
            glBegin(GL_POINTS);
            glColor3f(1.0, 1.0, 1.0);   // vert color
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
            switch (key)
            {
                /// gravity control
                case 'g':
                { g /= 1.1; gravity->update_parms("g", g);  std::cout << "gravity constant: " << g << std::endl; break; }
                case 'G':
                { g *= 1.1; gravity->update_parms("g", g);  std::cout << "gravity constant: " << g << std::endl; break; }

                /// timestep control
                case 't':
                { dt /= 1.1; std::cout << "time step " << dt << std::endl; break;}
                case 'T':
                { dt *= 1.1; std::cout << "time step " << dt << std::endl; break;}

                /// quit
                case 27: { exit(0); }

                default:
                    break;
            }
        }

        void Usage()
        {

        }

    private:
        // cloth and plane
        SoftBodyState SB;
        GeometryPtr geom;
        double size;
        // forces
        ForcePtrContainer forces;
        ForcePtr gravity;
        ForcePtr clothInner;
        // solver
        SolverPtr solver;

        /// keyboard controls
        int plane_div;
        int cloth_div;
        float g;    // gravity constant

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
                             std::vector<std::pair<size_t, size_t >>& edge_pairs)
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
                }
            }
        }
    };

    pba::PbaThing ClothInHole() { return PbaThing(new pba::ClothInHoleThing()); }

}

#endif //PBA_CLOTHTHING_H

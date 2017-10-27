//
// Created by jingcoz on 10/26/17.
//

#ifndef PBA_CLOTHTHING_H
#define PBA_CLOTHTHING_H

# include "PbaThing.h"
# include "Tools.h"
# include <algorithm>

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
                size(6.0),
                plane_div(5)    // be odd number
        {}
        ~ClothInHoleThing() {}

        void Init(const std::vector<std::string>& args)
        {
            /// load plane
            geom = CreateGeometry("collisionPlane");
            LoadMesh::LoadPlane(Vector(0.0, -3.0, 0.0), size, plane_div, geom);

            createHole(3);

            int i = 1;
            for (auto it = geom->get_triangles().begin(); it != geom->get_triangles().end(); ++it)
            {
                TrianglePtr triangle = *it;
                // triangle->setColor(Color(float(drand48() * 0.4 + 0.6), 0.5, float(drand48() * 0.4 + 0.6), 1.0));   // set random colors
                float a = 0.05 * i;
                triangle->setColor(Color(1.0 * a, 1.0 * a, 1.0 * a, 1.0));   // set random colors
                i++;
            }
            std::cout << "-------------------------------------------" << std::endl;
        }

        void Reset()
        {

        }

        void solve()
        {

        }

        void Display()
        {
            /// draw plane
            Draw::DrawTriangles(geom);
        }

        void Keyboard(unsigned char key, int x, int y)
        {
            switch (key)
            {
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
        GeometryPtr geom;
        double size;
        int plane_div;

        void createHole(const int& division_num)
        {
            cout << "size: " << geom->get_nb() << endl;
            int low = (plane_div - division_num) / 2;
            int up = (plane_div + division_num) / 2;
            for (int i = low; i <= up; ++i)
            {
                for (int j = low; j <= up; ++j)
                {
                    int id = i * plane_div + j;
                    geom->get_triangles()[id * 2] = nullptr;
                    geom->get_triangles()[id * 2 + 1] = nullptr;
                }
            }

//            std::remove(geom->get_triangles().begin(), geom->get_triangles().end(), nullptr);
            // geom->get_triangles().erase(std::remove(geom->get_triangles().begin(), geom->get_triangles().end(), nullptr));
            cout << "size: " << geom->get_nb() << endl;
        }
    };

    pba::PbaThing ClothInHole() { return PbaThing(new pba::ClothInHoleThing()); }

}

#endif //PBA_CLOTHTHING_H

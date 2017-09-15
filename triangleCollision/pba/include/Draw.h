//
// Created by jingcoz on 9/14/17.
//

# ifndef PBA_DRAW_H
# define PBA_DRAW_H

# ifdef __APPLE__
#include <OpenGL/gl.h>   // OpenGL itself.
  #include <OpenGL/glu.h>  // GLU support library.
  #include <GLUT/glut.h>
# else
# include <GL/gl.h>   // OpenGL itself.
# include <GL/glu.h>  // GLU support library.
# include <GL/glut.h> // GLUT support library.
# endif

# include <vector>
# include "Vector.h"
# include "Color.h"


namespace pba
{
    class Draw
    {
    public:
        Draw()  {}
        ~Draw() {}

        void draw_scene();

    private:
        std::vector<Vector> vertices;
        std::vector<Vector> normals;
        std::vector<Color> colors;
        std::vector<int> faces;
    };
}


# endif //PBA_DRAW_H

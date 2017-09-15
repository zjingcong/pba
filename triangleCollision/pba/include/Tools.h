//
// Created by jingcoz on 9/14/17.
//

# ifndef PBA_TOOLS_H
# define PBA_TOOLS_H

# ifdef __APPLE__
  # include <OpenGL/gl.h>   // OpenGL itself.
  # include <OpenGL/glu.h>  // GLU support library.
  # include <GLUT/glut.h>
# else
# include <GL/gl.h>   // OpenGL itself.
# include <GL/glu.h>  // GLU support library.
# include <GL/glut.h> // GLUT support library.
# endif

# include <vector>
# include <string>
# include "Vector.h"
# include "Color.h"


namespace pba
{

    class Draw
    {
    public:
        //! draw triangles for verts, face_indices and face_colors
        static void DrawTriangles(std::vector<Vector> vertices, std::vector<Vector> face_indices, std::vector<Color> face_colors);
    };


    class LoadMesh
    {
    public:
        //! load obj file mesh
        static void LoadObj(std::string obj_path, std::vector<Vector>& vertices, std::vector<Vector>& face_indices);
        //! load simple box with 12 triangles
        static void LoadBox(const float l, std::vector<Vector>& vertices, std::vector<Vector>& face_indices);
    };
}


# endif //PBA_TOOLS_H

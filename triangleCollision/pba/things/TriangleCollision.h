//
// Created by jingcoz on 9/13/17.
//

#ifndef PBA_TRIANGLECOLLISION_H
#define PBA_TRIANGLECOLLISION_H

#include "PbaThing.h"
#include "Vector.h"
#include "Color.h"

#ifdef __APPLE__
#include <OpenGL/gl.h>   // OpenGL itself.
  #include <OpenGL/glu.h>  // GLU support library.
  #include <GLUT/glut.h>
#else
#include <GL/gl.h>   // OpenGL itself.
#include <GL/glu.h>  // GLU support library.
#include <GL/glut.h> // GLUT support library.
#endif

#include <iostream>

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
        TriangleCollisionThing(const std::string nam = "TriangleCollisionThing"): PbaThingyDingy(nam) {}
        ~TriangleCollisionThing()   {}

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
                { dt /= 1.1; std::cout << "time step " << dt << std::endl; }
                case 'T':
                { dt *= 1.1; std::cout << "time step " << dt << std::endl; }

                // particles number control

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
        }

    private:
        // keyboard selection
        float g;    // gravity constant
        float dt;   // timestep
        float num;  // num of particles
        float Cr;   // coefficient of restitution
        float Cs;   // coefficient of stickness
    };

}

#endif //PBA_TRIANGLECOLLISION_H

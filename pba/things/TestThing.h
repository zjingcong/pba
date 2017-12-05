//
// Created by jingcoz on 11/22/17.
//

#ifndef PBA_SPHERESTHING_H
#define PBA_SPHERESTHING_H

# include "PbaThing.h"
# include "DynamicalState.h"

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

    class TestThing: public PbaThingyDingy
    {
    public:
        TestThing(const std::string nam = "UnitTestThing") :
                PbaThingyDingy(nam)
        {
            dt = 1.0/24;
            DS = CreateDynamicalState("unitTestDS");
            std::cout << "unit test thing" << std::endl;
        }

        ~TestThing() {}

        void Init(const std::vector<std::string> &args)
        {
            add_particles(10);
            std::cout << "current nb: " << DS->nb() << std::endl;
            print_id();
            std::cout << "Delete 2" << std::endl;
            DS->remove(2);
            std::cout << "current nb: " << DS->nb() << std::endl;
            print_id();
        }

        void Reset()
        {
        }

        void solve()
        {
        }

        void Display()
        {
        }

        void Keyboard(unsigned char key, int x, int y)  {}

        void Usage()    {}

    private:
        DynamicalState DS;

        void add_particles(size_t particle_num)
        {
            size_t nb = DS->nb();
            DS->add(particle_num);
            for (size_t i = 0; i < particle_num; ++i)
            {
                size_t id = nb + i;
                DS->set_id(id, int(id));
            }
        }

        void print_id()
        {
            for (size_t i = 0; i < DS->nb(); ++i)
            {
                std::cout << "id: " << DS->id(i) << " ";
            }
            std::cout << std::endl;
        }
    };

    pba::PbaThing Tests() { return PbaThing(new pba::TestThing()); }

}

#endif //PBA_SPHERESTHING_H

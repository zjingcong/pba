//
// Created by jingcoz on 9/14/17.
//

#ifndef PBA_UNITTEST_H
#define PBA_UNITTEST_H

# include "PbaThing.h"
# include "Vector.h"
# include "Color.h"
# include "Solver.h"
# include "Force.h"
# include "DynamicalState.h"
# include "Tools.h"
# include "Geometry.h"
# include "Collision.h"

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

using namespace std;

namespace pba
{

    class unitTestThing: public PbaThingyDingy
    {
    public:
        unitTestThing(const std::string nam = "unitTest"): PbaThingyDingy(nam) {}
        ~unitTestThing() {}

        void Init(const std::vector<std::string>& args)
        {
//            // test argvs
//            for (auto it = args.cbegin(); it != args.cend(); ++it)
//            {
//                std::cout << *it << std::endl;
//            }
//
            // testObj();
            geom = new TriangleGeometry("box");
            testBox();

//            if (testCollisionDetection())   {cout << "collision" << endl;}
//            else    {cout << "no collision" << endl;}

            cout << "================solverTest=============" << endl;
            // testBaseSolver();
            // testSolver();
//            testCollisionLF();
            cout << "=============solverTestEND=============" << endl;
        }


        void Display()
        {
//            testDrawTriangles();
        }


    private:
        GeometryPtr geom;

        void testBaseSolver()
        {
//            SolverPtr solver;
//            solver = new SolverBase();
//
//            cout << "solver name: " << solver->Name() << endl;
//
//            DynamicalState DS = CreateDynamicalState("testDs");
//            DS->add();
//            DS->set_pos(0, Vector(0.0, 0.0, 0.0));
//            DS->set_id(0, 0);
//            DS->set_mass(0, 1);
//            DS->set_vel(0, Vector(0.0, 0.0, 0.0));
//            cout << "DS item number: " << DS->nb() << endl;
//
//            ForcePtr gravity = new Gravity(9.8);
//            cout << "Gravity.Y: " << gravity->getForce(DS, 0).Y() << endl;
//
//            solver->setForce(gravity);
//            solver->setDS(DS);
//
//            cout << "pos.Y: " << DS->pos(0).Y() << endl;
//            cout << "update pos" << endl;
//            // solver->updatePos(float(1.0/24.0));
//            cout << "pos.Y: " << DS->pos(0).Y() << endl;
//            cout << "update vel" << endl;
//            // solver->updateVel(float(1.0/24.0));
//            cout << "vel.Y: " << DS->vel(0).Y() << endl;
        }

        void testSolver()
        {
//            SolverPtr solver;
//            solver = new LeapFrogSolver();
//
//            cout << "solver name: " << solver->Name() << endl;
//
//            DynamicalState DS = CreateDynamicalState("testDs");
//            DS->add();
//            DS->set_pos(0, Vector(0.0, 0.0, 0.0));
//            DS->set_id(0, 0);
//            DS->set_mass(0, 1);
//            DS->set_vel(0, Vector(0.0, 0.0, 0.0));
//            cout << "DS item number: " << DS->nb() << endl;
//
//            ForcePtr gravity = new Gravity(9.8);
//            cout << "Gravity.Y: " << gravity->getForce(DS, 0).Y() << endl;
//
//            solver->setForce(gravity);
//            solver->setDS(DS);
//
//            cout << "pos.Y: " << DS->pos(0).Y();
//            cout << " vel.Y: " << DS->vel(0).Y() << endl;
//            cout << "update ds" << endl;
//            solver->updateDS(float(1.0 / 24.0));
//            cout << "pos.Y: " << DS->pos(0).Y();
//            cout << " vel.Y: " << DS->vel(0).Y() << endl;
        }

        void testObj()
        {
            LoadMesh::LoadObj("/home/jingcoz/workspace/pba/hw1/models/bigsphere.obj", geom);
            cout << "triangle nb: " << geom->get_nb() << endl;
        }

        void testBox()
        {
            LoadMesh::LoadBox(10, geom);
            cout << "triangle nb: " << geom->get_nb() << endl;
            for (auto it = geom->get_triangles().cbegin(); it != geom->get_triangles().cend(); ++it)
            {
                // collision detection
                TrianglePtr triangle = *it;
                cout << "triangle P0" << triangle->getP0().X()  << triangle->getP0().Y() << triangle->getP0().Z() << endl;
                cout << "triangle P1" << triangle->getP1().X()  << triangle->getP1().Y() << triangle->getP1().Z() << endl;
                cout << "triangle P2" << triangle->getP2().X()  << triangle->getP2().Y() << triangle->getP2().Z() << endl;
                cout << "triangle norm" << triangle->getNorm().X()  << triangle->getNorm().Y() << triangle->getNorm().Z() << endl;
            }
        }

        void testDrawTriangles()
        {
            for (size_t i = 0; i < geom->get_nb(); ++i)
                { geom->add_face_color(Color(float(drand48()), float(drand48()), float(drand48()), 1.0)); } // set random colors
            Draw::DrawTriangles(geom);
        }

        bool testCollisionDetection()
        {
            Vector x_i;

            Vector p = Vector(0.2, -0.9, 0.2);
            Vector vel = Vector(0.0, -1.0, 0.0);
            TrianglePtr triangle = new Triangle(Vector(0, 0, 0), Vector(1, 0, 0), Vector(0, 0, 1));
            double d_t = 1.0;


            // cout << "collisionDetection" << endl;
            // calculate dt_i and x_i
            cout << "norm: " << triangle->getNorm().Y() << endl;
            double dti = ((p - triangle->getP0()) * triangle->getNorm()) / (vel * triangle->getNorm());
            cout << "dti: " << dti << endl;
            x_i = p - vel * dti;
            cout << "x_i: " << x_i.X() << x_i.Y() << x_i.Z() << endl;

            // collision detection
            Vector e1 = triangle->getE1();
            Vector e2 = triangle->getE2();
            double a = (e2^e1) * (e2^(x_i - triangle->getP0())) / pow((e2^e1).magnitude(), 2.0);
            if (a < 0 || a > 1) {cout << "0" << endl;return false;}
            double b = (e1^e2) * (e1^(x_i - triangle->getP0())) / pow((e1^e2).magnitude(), 2.0);
            if (b < 0 || b > 1) {cout << "1" << endl;return false;}
            double c = a + b;
            if (c < 0 || c > 1) {cout << "2" << endl;return false;}

            if ((d_t * dti) < 0)    {cout << "3" << endl;return false;}
            if (fabs(dti) > fabs(d_t))  {cout << "4" << endl;return false;}
            if ((d_t - dti) / d_t < pow(10.0, -6.0)) {cout << "5" << endl;return false;}

            // pick the maximum dt_i, and set the collision norm for handling calculation
//            if (dti > dt_i)
//            {
//                dt_i = dti;
//                collision_triangle = triangle;
//            }

            return true;
        }

        void testCollisionLF()
        {
            SolverPtr solver;
            solver = new LeapFrogSolver();
            cout << "solver name: " << solver->Name() << endl;

            DynamicalState DS = CreateDynamicalState("testDs");
            DS->add();
            DS->set_pos(0, Vector(0.0, 0.0, 0.0));
            DS->set_id(0, 0);
            DS->set_mass(0, 1);
            DS->set_vel(0, Vector(0.0, 0.0, 0.0));
            cout << "DS item number: " << DS->nb() << endl;

            ForcePtr gravity = new Gravity(9.8);
            cout << "Gravity.Y: " << gravity->getForce(DS, 0).Y() << endl;

            cout << "pos.Y: " << DS->pos(0).Y();
            cout << " vel.Y: " << DS->vel(0).Y() << endl;
            cout << "update ds" << endl;
            // solver->updateDSWithCollision(1.0/24.0, DS, gravity, collisionMesh);
            solver->updateDS(1.0/24.0, DS, gravity);
            cout << "pos.Y: " << DS->pos(0).Y();
            cout << " vel.Y: " << DS->vel(0).Y() << endl;
        }
    };

    pba::PbaThing unitTest(){ return PbaThing( new pba::unitTestThing() ); }
}

#endif //PBA_UNITTEST_H

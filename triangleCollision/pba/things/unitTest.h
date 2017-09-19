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

            cout << "================solverTest=============" << endl;
            // testBaseSolver();
            // testSolver();
            cout << "=============solverTestEND=============" << endl;
        }


        void Display()
        {
            testDrawTriangles();
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
        }

        void testDrawTriangles()
        {
            for (size_t i = 0; i < geom->get_nb(); ++i)
                { geom->add_face_color(Color(float(drand48()), float(drand48()), float(drand48()), 1.0)); } // set random colors
            Draw::DrawTriangles(geom);
        }
    };

    pba::PbaThing unitTest(){ return PbaThing( new pba::unitTestThing() ); }
}

#endif //PBA_UNITTEST_H

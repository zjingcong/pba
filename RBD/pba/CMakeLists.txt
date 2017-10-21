cmake_minimum_required(VERSION 3.6)
project(pba)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        base/BBox.C
        base/Boid.C
        base/Collision.C
        base/DynamicalState.C
        base/Force.C
        base/Geometry.C
        base/KdTree.C
        base/LinearAlgebra.C
        base/Matrix.C
        base/PbaThing.C
        base/PbaViewer.C
        base/Solver.C
        base/Tools.C
        base/Triangle.C
        include/BBox.h
        include/Boid.h
        include/Collision.h
        include/Color.h
        include/DynamicalState.h
        include/Force.h
        include/Geometry.h
        include/KdTree.h
        include/LinearAlgebra.h
        include/Matrix.h
        include/PbaThing.h
        include/PbaViewer.h
        include/Solver.h
        include/Tools.h
        include/Triangle.h
        include/Vector.h
        things/BoidFlocking.h things/RBDThing.h include/RigidBodyState.h base/RigidBodyState.C include/RBD.h base/RBD.C)

include_directories(include/)

add_executable(pba ${SOURCE_FILES})